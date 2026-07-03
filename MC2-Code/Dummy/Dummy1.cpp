#include <SPI.h>
#include <ArduinoBLE.h>
#include <arm_math.h>

// =============================
// SYSTEM CONSTANTS
// =============================

#define CHANNELS 14
#define MUX_CHANNELS 7

// Pin configuration
const int CS1_PIN   = 10;
const int CS2_PIN   = 9;
const int DRDY1_PIN = 2;
const int DRDY2_PIN = 3;
const int START_PIN = 4;
const int RESET_PIN = 5;

// ADS1248 MUX configurations
const byte MUX_CONFIGS[MUX_CHANNELS] =
{
  0x07,0x0F,0x17,0x1F,0x27,0x2F,0x37
};

// SPI configuration
SPISettings adcSPISettings(2000000, MSBFIRST, SPI_MODE1);

// =============================
// INTERRUPT FLAGS
// =============================

volatile bool drdy1_flag=false;
volatile bool drdy2_flag=false;

// =============================
// DATA BUFFERS
// =============================

int32_t rawChannels[CHANNELS];

float32_t dspOutput[CHANNELS];

int16_t compressedData[CHANNELS];

// BLE packet
// 4 bytes timestamp + 14*2 bytes EEG = 32 bytes
byte blePacket[32];

// =============================
// DSP FILTERS
// =============================

// 50Hz notch filter coefficients
float32_t notchCoeffs[5] =
{
  0.972233,
 -1.847759,
  0.972233,
  1.847759,
 -0.944466
};

float32_t notchState[CHANNELS][4];

arm_biquad_casd_df1_inst_f32 notchFilter[CHANNELS];

// =============================
// DC REMOVAL FILTER
// =============================

float32_t alpha=0.995;

float32_t prevInput[CHANNELS];
float32_t prevOutput[CHANNELS];

float32_t removeDC(int ch,float32_t x)
{
  float32_t y = x - prevInput[ch] + alpha*prevOutput[ch];

  prevInput[ch]=x;
  prevOutput[ch]=y;

  return y;
}

// =============================
// BLE SERVICE
// =============================

BLEService eegService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLECharacteristic eegCharacteristic(
"19B10001-E8F2-537E-4F6C-D104768A1214",
BLENotify,
32
);

// =============================
// INTERRUPTS
// =============================

void drdy1_ISR()
{
  drdy1_flag=true;
}

void drdy2_ISR()
{
  drdy2_flag=true;
}

// =============================
// SETUP
// =============================

void setup()
{

  Serial.begin(115200);

  pinMode(CS1_PIN,OUTPUT);
  pinMode(CS2_PIN,OUTPUT);
  pinMode(START_PIN,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);

  pinMode(DRDY1_PIN,INPUT_PULLUP);
  pinMode(DRDY2_PIN,INPUT_PULLUP);

  digitalWrite(CS1_PIN,HIGH);
  digitalWrite(CS2_PIN,HIGH);
  digitalWrite(START_PIN,LOW);

  // hardware reset
  digitalWrite(RESET_PIN,LOW);
  delay(10);
  digitalWrite(RESET_PIN,HIGH);
  delay(50);

  SPI.begin();

  if(!BLE.begin())
  {
    while(1);
  }

  BLE.setLocalName("SmartBCI");
  BLE.setAdvertisedService(eegService);

  eegService.addCharacteristic(eegCharacteristic);

  BLE.addService(eegService);

  BLE.advertise();

  // initialize filters
  for(int i=0;i<CHANNELS;i++)
  {
    arm_biquad_cascade_df1_init_f32(
      &notchFilter[i],
      1,
      notchCoeffs,
      notchState[i]
    );
  }

  // attach interrupts
  attachInterrupt(
  digitalPinToInterrupt(DRDY1_PIN),
  drdy1_ISR,
  FALLING
  );

  attachInterrupt(
  digitalPinToInterrupt(DRDY2_PIN),
  drdy2_ISR,
  FALLING
  );

  digitalWrite(START_PIN,HIGH);

  // initial MUX setup
  setMultiplexer(CS1_PIN,MUX_CONFIGS[0]);
  setMultiplexer(CS2_PIN,MUX_CONFIGS[0]);
}

// =============================
// MAIN LOOP
// =============================

int muxIndex=0;

void loop()
{

  BLEDevice central=BLE.central();

  bool d1,d2;

  noInterrupts();
  d1=drdy1_flag;
  d2=drdy2_flag;
  drdy1_flag=false;
  drdy2_flag=false;
  interrupts();

  if(d1 && d2)
  {

    rawChannels[muxIndex]     = readADC(CS1_PIN);
    rawChannels[muxIndex +7 ] = readADC(CS2_PIN);

    muxIndex++;

    if(muxIndex < 7)
    {

      setMultiplexer(CS1_PIN,MUX_CONFIGS[muxIndex]);
      setMultiplexer(CS2_PIN,MUX_CONFIGS[muxIndex]);

    }
    else
    {

      muxIndex=0;

      setMultiplexer(CS1_PIN,MUX_CONFIGS[0]);
      setMultiplexer(CS2_PIN,MUX_CONFIGS[0]);

      if(central && central.connected())
      {
        processFrame();
      }

    }

  }

}

// =============================
// READ ADC DATA
// =============================

int32_t readADC(int csPin)
{

  SPI.beginTransaction(adcSPISettings);

  digitalWrite(csPin,LOW);

  SPI.transfer(0x12);

  byte b1=SPI.transfer(0xFF);
  byte b2=SPI.transfer(0xFF);
  byte b3=SPI.transfer(0xFF);

  digitalWrite(csPin,HIGH);

  SPI.endTransaction();

  int32_t value=(b1<<16)|(b2<<8)|b3;

  if(value & 0x800000)
  value |= 0xFF000000;

  return value;

}

// =============================
// SET MULTIPLEXER
// =============================

void setMultiplexer(int csPin,byte muxConfig)
{

  SPI.beginTransaction(adcSPISettings);

  digitalWrite(csPin,LOW);

  SPI.transfer(0x40);
  SPI.transfer(0x00);
  SPI.transfer(muxConfig);

  SPI.transfer(0xFC); // SYNC
  SPI.transfer(0x00); // WAKEUP

  digitalWrite(csPin,HIGH);

  SPI.endTransaction();

}

// =============================
// PROCESS FRAME + BLE
// =============================

void processFrame()
{

  uint32_t timestamp=micros();

  int index=0;

  blePacket[index++] = timestamp >> 24;
  blePacket[index++] = timestamp >> 16;
  blePacket[index++] = timestamp >> 8;
  blePacket[index++] = timestamp;

  for(int i=0;i<CHANNELS;i++)
  {

    float32_t x=(float32_t)rawChannels[i];

    x=removeDC(i,x);

    arm_biquad_cascade_df1_f32(
      &notchFilter[i],
      &x,
      &dspOutput[i],
      1
    );

    int16_t val = dspOutput[i] >> 8;

    blePacket[index++] = val >> 8;
    blePacket[index++] = val;

  }

  eegCharacteristic.writeValue(blePacket,32);

}