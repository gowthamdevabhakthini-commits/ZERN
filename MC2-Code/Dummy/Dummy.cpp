#include <SPI.h>
#include <ArduinoBLE.h>
#include <arm_math.h>

// ==========================================
// 1. PIN DEFINITIONS & HARDWARE STATES
// ==========================================
const int CS1_PIN   = 0; // ADC 1 Chip Select (Active Low)
const int CS2_PIN   = 1; // ADC 2 Chip Select (Active Low)
const int DRDY1_PIN = 2; // ADC 1 Data Ready (Active Low)
const int DRDY2_PIN = 3; // ADC 2 Data Ready (Active Low)
const int START_PIN = 4; // Shared Start (Active High)
const int RESET_PIN = 5; // Shared Reset (Active Low)

// ==========================================
// 2. MULTIPLEXER & DATA ARRAYS
// ==========================================
// The 7 MUX commands to cycle AIN0-AIN6 against the AIN7 Reference
const byte MUX_CONFIGS[7] = {0x07, 0x0F, 0x17, 0x1F, 0x27, 0x2F, 0x37};
volatile int currentMuxIndex = 0;

// Data arrays for the 14 active sensors
int32_t rawChannels[14];     
float32_t dspInputArray[14]; 
float32_t dspOutputArray[14]; 
byte blePayload[42]; // 14 channels * 3 bytes (24-bit)

// Interrupt flags
volatile bool drdy1_Triggered = false;
volatile bool drdy2_Triggered = false;

// SPI Settings for ADS1248 (2 MHz, MSB First, SPI Mode 1)
SPISettings adcSPISettings(2000000, MSBFIRST, SPI_MODE1);

// ==========================================
// 3. BLUETOOTH LOW ENERGY (BLE) SETUP
// ==========================================
BLEService bciService("19B10000-E8F2-537E-4F6C-D104768A1214"); 
// 42-byte payload characteristic for Notification
BLECharacteristic bciDataChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 42); 

// ==========================================
// 4. DSP FILTER SETUP (CMSIS-DSP)
// ==========================================
// Placeholder coefficients for a 50Hz IIR Notch Filter at your specific sample rate
// You must generate these exact float values using Python (scipy.signal.iirnotch) or MATLAB
float32_t iirCoeffs[5] = {1.0, -1.9, 1.0, 1.9, -0.9}; 
float32_t iirState[14][4]; // State buffers for 14 independent channels
arm_biquad_casd_df1_inst_f32 notchFilter[14];

void setup() {
  Serial.begin(115200);

  // --- Initialize Digital Pins ---
  pinMode(CS1_PIN, OUTPUT);
  pinMode(CS2_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  pinMode(START_PIN, OUTPUT);
  pinMode(DRDY1_PIN, INPUT_PULLUP);
  pinMode(DRDY2_PIN, INPUT_PULLUP);

  // --- Establish Idle States ---
  digitalWrite(CS1_PIN, HIGH);
  digitalWrite(CS2_PIN, HIGH);
  digitalWrite(START_PIN, LOW);

  // --- Hardware Reset Sequence ---
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(50); // Allow internal 2.048MHz oscillator to stabilize

  // --- Initialize SPI & BLE ---
  SPI.begin();
  if (!BLE.begin()) {
    while (1); // Halt if Bluetooth fails to initialize
  }

  BLE.setLocalName("SmartGlasses_BCI");
  BLE.setAdvertisedService(bciService);
  bciService.addCharacteristic(bciDataChar);
  BLE.addService(bciService);
  BLE.advertise();
  //2. Increase BLE Throughput (Huge Improvement)
  //BLE.setConnectionInterval(6, 6);
  //BLE.setSupervisionTimeout(100);

  // --- Initialize DSP Filters ---
  for(int i = 0; i < 14; i++) {
    arm_biquad_cascade_df1_init_f32(&notchFilter[i], 1, iirCoeffs, iirState[i]);
  }

  // --- Start Conversion Cycle ---
  digitalWrite(START_PIN, HIGH); // Pull HIGH to sync both chips
  
  // Attach interrupts for the Active-Low DRDY pins
  attachInterrupt(digitalPinToInterrupt(DRDY1_PIN), drdy1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(DRDY2_PIN), drdy2_ISR, FALLING);
  
  // Set initial multiplexer state
  setMultiplexer(CS1_PIN, MUX_CONFIGS[0]);
  setMultiplexer(CS2_PIN, MUX_CONFIGS[0]);
}

// ==========================================
// 5. INTERRUPT SERVICE ROUTINES
// ==========================================
void drdy1_ISR() { drdy1_Triggered = true; }
void drdy2_ISR() { drdy2_Triggered = true; }

// ==========================================
// 6. MAIN EXECUTION LOOP
// ==========================================
void loop() {
  BLEDevice central = BLE.central();

  // When both chips indicate the current MUX channel is ready
  if (drdy1_Triggered && drdy2_Triggered) {
    drdy1_Triggered = false;
    drdy2_Triggered = false;

    // Read the 24-bit data from the current MUX channel
    rawChannels[currentMuxIndex]     = readADCData(CS1_PIN);      // ADC 1 (Sensors 1-7)
    rawChannels[currentMuxIndex + 7] = readADCData(CS2_PIN);      // ADC 2 (Sensors 8-14)

    // Increment multiplexer to the next channel
    currentMuxIndex++;
    
    if (currentMuxIndex < 7) {
      // Command the ADCs to switch to the next pin
      setMultiplexer(CS1_PIN, MUX_CONFIGS[currentMuxIndex]);
      setMultiplexer(CS2_PIN, MUX_CONFIGS[currentMuxIndex]);
    } 
    else {
      // A full 14-channel frame has been collected
      currentMuxIndex = 0; 
      setMultiplexer(CS1_PIN, MUX_CONFIGS[0]); // Reset to AIN0
      setMultiplexer(CS2_PIN, MUX_CONFIGS[0]);

      // --- Process & Transmit the Frame ---
      if (central && central.connected()) {
        processAndTransmitData();
      }
    }
  }
}

// ==========================================
// 7. HARDWARE CONTROL FUNCTIONS
// ==========================================
void setMultiplexer(int csPin, byte muxConfig) {
  SPI.beginTransaction(adcSPISettings);
  digitalWrite(csPin, LOW);
  
  SPI.transfer(0x40);      // WREG command for MUX0 register
  SPI.transfer(0x00);      // Write 1 register
  SPI.transfer(muxConfig); // Apply AINx / AIN7 config
  //1.Improve ADC Channel Switching Accuracy______BY_CHATGPT
  // SPI.transfer(0xFC);      // SYNC
  //SPI.transfer(0x00);      // WAKEUP
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

int32_t readADCData(int csPin) {
  SPI.beginTransaction(adcSPISettings);
  digitalWrite(csPin, LOW);
  
  SPI.transfer(0x12); // Send RDATA command
  
  // Read 3 bytes (24 bits)
  byte b1 = SPI.transfer(0xFF);
  byte b2 = SPI.transfer(0xFF);
  byte b3 = SPI.transfer(0xFF);
  
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  // Reconstruct 24-bit word
  int32_t adcValue = (b1 << 16) | (b2 << 8) | b3;
  
  // 24-bit Sign Extension (Two's Complement)
  if (adcValue & 0x800000) {
    adcValue |= 0xFF000000;
  }
  
  return adcValue;
}

// ==========================================
// 8. DSP & BLUETOOTH TRANSMISSION
// ==========================================
void processAndTransmitData() {
  int payloadIndex = 0;

  for (int i = 0; i < 14; i++) {
    // 1. Cast integer data to float for ARM CMSIS-DSP
    dspInputArray[i] = (float32_t)rawChannels[i];

    // 2. Apply Digital Filtering (50Hz Notch)
    arm_biquad_cascade_df1_f32(&notchFilter[i], &dspInputArray[i], &dspOutputArray[i], 1);

    // 3. Cast back to 32-bit integer
    int32_t cleanedValue = (int32_t)dspOutputArray[i];

    // 4. Pack into BLE Byte Array (3 bytes per channel)
    blePayload[payloadIndex++] = (cleanedValue >> 16) & 0xFF;
    blePayload[payloadIndex++] = (cleanedValue >> 8) & 0xFF;
    blePayload[payloadIndex++] = cleanedValue & 0xFF;
  }

  // 5. Push the unified 42-byte array to the mobile phone
  bciDataChar.writeValue(blePayload, 42);
}