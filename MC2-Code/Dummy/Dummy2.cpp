#include <SPI.h>
#include <ArduinoBLE.h>
#include <arm_math.h>
static constexpr int PIN_CS1    = D0;   // P0.02
static constexpr int PIN_CS2    = D1;   // P0.03
static constexpr int PIN_DRDY1  = D2;   // P0.28 — interrupt capable
static constexpr int PIN_DRDY2  = D3;   // P0.29 — interrupt capable
static constexpr int PIN_START  = D4;   // P0.04
static constexpr int PIN_RESET  = D5;   // P0.05
static constexpr uint8_t ADS_WAKEUP   = 0x00; // Exit power-down
static constexpr uint8_t ADS_SLEEP    = 0x02; // Enter power-down
static constexpr uint8_t ADS_SYNC1    = 0x04; // SYNC byte 1 (must send twice)
static constexpr uint8_t ADS_SYNC2    = 0x04; // SYNC byte 2
static constexpr uint8_t ADS_RESET    = 0x06; // Reset registers to default
static constexpr uint8_t ADS_NOP      = 0xFF; // No operation
static constexpr uint8_t ADS_RDATA    = 0x12; // Read single conversion result
static constexpr uint8_t ADS_RDATAC   = 0x14; // Read data continuously (power-on default)
static constexpr uint8_t ADS_SDATAC   = 0x16; // Stop read data continuous mode
static constexpr uint8_t ADS_WREG     = 0x40; // Write register(s): OR with address
static constexpr uint8_t ADS_RREG     = 0x20; // Read register(s): OR with address
static constexpr uint8_t ADS_SELFOCAL = 0x62; // Self offset calibration
static constexpr uint8_t REG_MUX0  = 0x00; // Mux control 0: BCS, MUX_SP, MUX_SN
static constexpr uint8_t REG_VBIAS = 0x01; // Bias voltage enable per AIN pin
static constexpr uint8_t REG_MUX1  = 0x02; // Mux control 1: CLKSTAT, VREFCON, REFSELT, MUXCAL
static constexpr uint8_t REG_SYS0  = 0x03; // PGA gain + data rate
static constexpr uint8_t REG_IDAC0 = 0x0A; // IDAC magnitude + DRDYMODE
static constexpr uint8_t REG_IDAC1 = 0x0B; // IDAC output routing
static constexpr uint8_t VAL_SYS0 = (6 << 4) | 9;   // 0x69: PGA=64, DR=2000 SPS
static constexpr uint8_t VAL_MUX1 = 0x30;
static constexpr uint8_t VAL_VBIAS = 0x80;  // bias on AIN7 reference
static constexpr uint8_t VAL_IDAC0 = 0x00;
static constexpr uint8_t VAL_IDAC1 = 0xFF;
static constexpr uint8_t MUX_CONFIGS[7] = { 
    0x07,   // Channel 0: AIN0 vs AIN7 
    0x0F,   // Channel 1: AIN1 vs AIN7
    0x17,   // Channel 2: AIN2 vs AIN7
    0x1F,   // Channel 3: AIN3 vs AIN7
    0x27,   // Channel 4: AIN4 vs AIN7
    0x2F,   // Channel 5: AIN5 vs AIN7
    0x37    // Channel 6: AIN6 vs AIN7
};
static constexpr int NUM_MUX_CHANNELS = 7;
static constexpr int     NUM_CHANNELS    = 14;          // 7 per ADS1248
static constexpr int     BYTES_PER_CH    = 3;           // 24-bit raw value
static constexpr int     PAYLOAD_BYTES   = NUM_CHANNELS * BYTES_PER_CH;  // 42
static constexpr uint32_t WATCHDOG_MS    = 2000;        // Reset if no DRDY for 2 s
static const SPISettings ADS_SPI(2000000, MSBFIRST, SPI_MODE1);
static int32_t   rawData   [NUM_CHANNELS];              // Raw 24-bit ADC values
static float32_t filterIn  [NUM_CHANNELS];              // DSP input (float)
static float32_t filterOut [NUM_CHANNELS];              // DSP output (float)
static uint8_t   bleFrame  [PAYLOAD_BYTES];             // BLE transmit buffer
volatile bool     gDrdy1       = false;
volatile bool     gDrdy2       = false;
volatile bool     gFrameReady  = false;
volatile uint8_t  gMuxIndex    = 0;
volatile uint32_t gLastDrdyMs  = 0;
static float32_t iirCoeffs[5] = {
    0.9788f,    // b0
   -1.8744f,   // b1
    0.9788f,    // b2
    1.8744f,   // -a1  (CMSIS sign convention: negated)
   -0.9577f    // -a2  (CMSIS sign convention: negated)
};
static float32_t iirState[NUM_CHANNELS * 4];
static arm_biquad_casd_df1_inst_f32 iirFilter[NUM_CHANNELS];
static BLEService        bciService ("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLECharacteristic bciDataChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                                      BLERead | BLENotify,
                                      PAYLOAD_BYTES);
void drdy1_ISR() { gDrdy1 = true;  gLastDrdyMs = millis(); }
void drdy2_ISR() { gDrdy2 = true; }
static inline void ads_cmd(int cs, uint8_t cmd) {
    SPI.beginTransaction(ADS_SPI);
    digitalWrite(cs, LOW);
    SPI.transfer(cmd);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}
 * @param cs      Chip Select pin
 * @param reg     Starting register address
 * @param vals    Pointer to data bytes
 * @param count   Number of registers to write
static void ads_wreg(int cs, uint8_t reg, const uint8_t *vals, uint8_t count) {
    SPI.beginTransaction(ADS_SPI);
    digitalWrite(cs, LOW);
    SPI.transfer(ADS_WREG | (reg & 0x0F));  // command byte
    SPI.transfer((uint8_t)(count - 1));      // n-1 registers
    for (uint8_t i = 0; i < count; i++) {
        SPI.transfer(vals[i]);
    }
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}
static uint8_t ads_rreg(int cs, uint8_t reg) {
    SPI.beginTransaction(ADS_SPI);
    digitalWrite(cs, LOW);
    SPI.transfer(ADS_RREG | (reg & 0x0F));
    SPI.transfer(0x00);
    uint8_t val = SPI.transfer(ADS_NOP);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    return val;
}
static int32_t ads_read_data(int cs) {
    SPI.beginTransaction(ADS_SPI);
    digitalWrite(cs, LOW);
    SPI.transfer(ADS_RDATA);
    uint8_t b0 = SPI.transfer(ADS_NOP);   // MSB
    uint8_t b1 = SPI.transfer(ADS_NOP);   // middle byte
    uint8_t b2 = SPI.transfer(ADS_NOP);   // LSB
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    int32_t val = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | (int32_t)b2;
    if (val & 0x800000) {
        val |= 0xFF000000;
    }
    return val;
}
static inline void ads_set_channel(int cs, uint8_t mux0) {
    ads_wreg(cs, REG_MUX0, &mux0, 1);
}
static void ads_init(int cs) {
    ads_cmd(cs, ADS_SDATAC);
    delayMicroseconds(100);   // brief pause for command to take effect
    const uint8_t mux1 = VAL_MUX1;
    ads_wreg(cs, REG_MUX1, &mux1, 1);
    delayMicroseconds(200);   // reference stabilisation
    const uint8_t sys0 = VAL_SYS0;
    ads_wreg(cs, REG_SYS0, &sys0, 1);
    delayMicroseconds(100);
    const uint8_t vbias = VAL_VBIAS;
    ads_wreg(cs, REG_VBIAS, &vbias, 1);
    const uint8_t idac[2] = { VAL_IDAC0, VAL_IDAC1 };
    ads_wreg(cs, REG_IDAC0, idac, 2);
    ads_cmd(cs, ADS_SELFOCAL);
    uint32_t t0 = millis();
    while (digitalRead(cs == PIN_CS1 ? PIN_DRDY1 : PIN_DRDY2) == HIGH) {
        if (millis() - t0 > 50) {
            Serial.println("WARN: calibration timeout");
            break;
        }
    }
    delayMicroseconds(500);   // allow shift register to settle after DRDY
    const uint8_t mux0_ch0 = MUX_CONFIGS[0];
    ads_wreg(cs, REG_MUX0, &mux0_ch0, 1);
}
static void ads_hardware_reset_and_reinit() {
    detachInterrupt(digitalPinToInterrupt(PIN_DRDY1));
    detachInterrupt(digitalPinToInterrupt(PIN_DRDY2));
    digitalWrite(PIN_START, LOW);
    digitalWrite(PIN_RESET, LOW);
    delay(10);
    digitalWrite(PIN_RESET, HIGH);
    delay(5);
    ads_init(PIN_CS1);
    ads_init(PIN_CS2);
    digitalWrite(PIN_START, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_START, HIGH);
    noInterrupts();
    gDrdy1      = false;
    gDrdy2      = false;
    gFrameReady = false;
    gMuxIndex   = 0;
    gLastDrdyMs = millis();
    interrupts();
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY1), drdy1_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY2), drdy2_ISR, FALLING);
}
void setup() {
    Serial.begin(115200);
    pinMode(PIN_CS1,   OUTPUT);  digitalWrite(PIN_CS1,   HIGH);
    pinMode(PIN_CS2,   OUTPUT);  digitalWrite(PIN_CS2,   HIGH);
    pinMode(PIN_START, OUTPUT);  digitalWrite(PIN_START, LOW);
    pinMode(PIN_RESET, OUTPUT);  digitalWrite(PIN_RESET, HIGH);
    pinMode(PIN_DRDY1, INPUT_PULLUP);   // active-low signal
    pinMode(PIN_DRDY2, INPUT_PULLUP);
    digitalWrite(PIN_RESET, LOW);
    delay(10);
    digitalWrite(PIN_RESET, HIGH);
    delay(5);
    SPI.begin();
    if (!BLE.begin()) {
        Serial.println("FATAL: BLE init failed");
        while (1);
    }
    BLE.setLocalName("SmartGlasses_BCI");
    BLE.setAdvertisedService(bciService);
    bciService.addCharacteristic(bciDataChar);
    BLE.addService(bciService);
    BLE.advertise();
    Serial.println("BLE advertising as 'SmartGlasses_BCI'");
    for (int i = 0; i < NUM_CHANNELS; i++) {
        arm_biquad_cascade_df1_init_f32(
            &iirFilter[i],
            1,                  // numStages = 1 (single biquad)
            iirCoeffs,          // shared coefficient array (same filter, all channels)
            &iirState[i * 4]    // each channel has 4 state floats per stage
        );
    }
    ads_init(PIN_CS1);
    ads_init(PIN_CS2);
    digitalWrite(PIN_START, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_START, HIGH);
    gLastDrdyMs = millis();
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY1), drdy1_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY2), drdy2_ISR, FALLING);
    Serial.println("BCI firmware ready.");
}
void loop() {
    BLE.poll();
    if ((millis() - gLastDrdyMs) > WATCHDOG_MS) {
        Serial.println("WARN: DRDY watchdog — resetting ADCs");
        ads_hardware_reset_and_reinit();
        return;
    }
    if (gDrdy1 && gDrdy2) {
        noInterrupts();
        gDrdy1 = false;
        gDrdy2 = false;
        uint8_t idx = gMuxIndex;
        interrupts();
        rawData[idx]     = ads_read_data(PIN_CS1);  // Electrodes 1–7
        rawData[idx + 7] = ads_read_data(PIN_CS2);  // Electrodes 8–14
        if (idx < (NUM_MUX_CHANNELS - 1)) {
            uint8_t nextIdx = idx + 1;

            noInterrupts();
            gMuxIndex = nextIdx;
            interrupts();
            digitalWrite(PIN_START, LOW);
            ads_set_channel(PIN_CS1, MUX_CONFIGS[nextIdx]);
            ads_set_channel(PIN_CS2, MUX_CONFIGS[nextIdx]);
            digitalWrite(PIN_START, HIGH);
        } else {
            noInterrupts();
            gMuxIndex  = 0;
            gFrameReady = true;
            interrupts();
            digitalWrite(PIN_START, LOW);
            ads_set_channel(PIN_CS1, MUX_CONFIGS[0]);
            ads_set_channel(PIN_CS2, MUX_CONFIGS[0]);
            digitalWrite(PIN_START, HIGH);
        }
    }
    if (gFrameReady) {
        noInterrupts();
        gFrameReady = false;
        interrupts();
        BLEDevice central = BLE.central();
        if (central && central.connected()) {
            processAndTransmit();
        }
    }
}
static void processAndTransmit() {
    uint8_t payloadIdx = 0;
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        filterIn[ch] = (float32_t)rawData[ch];
        arm_biquad_cascade_df1_f32(
            &iirFilter[ch],
            &filterIn[ch],
            &filterOut[ch],
            1               // blockSize = 1 sample
        );
        int32_t cleaned = (int32_t)filterOut[ch];
        bleFrame[payloadIdx++] = (uint8_t)((cleaned >> 16) & 0xFF);
        bleFrame[payloadIdx++] = (uint8_t)((cleaned >>  8) & 0xFF);
        bleFrame[payloadIdx++] = (uint8_t)( cleaned        & 0xFF);
    }
    bciDataChar.writeValue(bleFrame, PAYLOAD_BYTES);
}
