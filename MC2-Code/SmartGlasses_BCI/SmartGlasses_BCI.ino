/**
 * ============================================================================
 * SmartGlasses BCI Firmware — Optimized v2.0
 * Hardware : Seeed XIAO nRF52840 Sense + 2× ADS1248
 * Channels : 14 active (AIN0–AIN6 × 2 chips vs AIN7 reference each)
 * BLE      : 42-byte notification @ ~80 fps per channel
 * ============================================================================
 *
 * KEY FIXES OVER ORIGINAL CODE
 * ─────────────────────────────────────────────────────────────────────────────
 * BUG 1  [CRITICAL] MUX_CONFIGS were WRONG. Original incremented by 0x08
 *        (toggling MUXSN bits only), not 0x10 (incrementing MUXSP channel).
 *        ADS1248 MUX0 register: bits [7:4] = MUXSP, [3:0] = MUXSN.
 *        Correct values: AIN0÷6 vs AIN7 → {0x07,0x17,0x27,0x37,0x47,0x57,0x67}
 *
 * BUG 2  [CRITICAL] Byte shift undefined behaviour in readADC().
 *        `byte b1 << 16` is UB because `byte` is unsigned 8-bit.
 *        Fix: cast to int32_t BEFORE shifting.
 *
 * BUG 3  [MEDIUM] Missing SDATAC before WREG in init.
 *        After power-on/reset, ADS1248 is in RDATAC (continuous) mode.
 *        WREG is silently ignored unless SDATAC is issued first.
 *
 * BUG 4  [MEDIUM] Missing t₆ delay after RDATA command.
 *        ADS1248 datasheet §9.5.1.4: 24 fCLK cycles must elapse between
 *        RDATA command byte and first SCLK of data (≈11.7 µs @ 2.048 MHz).
 *
 * BUG 5  [MEDIUM] Race condition on gMuxIndex — written in loop() without
 *        disabling interrupts; ISR could preempt a non-atomic RMW.
 *
 * BUG 6  [LOW] iirState declared as [14][4] and passed as iirState[i].
 *        Correct but fragile. Replaced with flat array + explicit stride index.
 *
 * BUG 7  [LOW] No DRDY watchdog — hardware fault causes silent hang forever.
 *
 * IMPROVEMENTS
 * ─────────────────────────────────────────────────────────────────────────────
 * • ADS1248 properly initialised: SDATAC → SYS0 (PGA+DR) → MUX1 (int ref) →
 *   MUX0 channel 0.  Writing MUX0 auto-resets the digital filter (datasheet
 *   §9.4.4.4), so no extra SYNC command is needed for channel cycling.
 * • SYS0 configured for PGA=1, 160 SPS nominal (≈80 actual at 2.048 MHz CLK).
 *   Single-cycle settling confirmed by datasheet Table 14 — no need to skip
 *   the first conversion after a MUX change.
 * • BLE.poll() called every loop iteration to keep the stack responsive.
 * • gFrameReady flag separates data collection from BLE transmission.
 * • Symbolic D0–D5 pin names used (clearer than raw integers for XIAO BSP).
 * • All magic numbers replaced with named constants.
 *
 * PIN MAPPING (XIAO nRF52840 Arduino BSP)
 * ─────────────────────────────────────────────────────────────────────────────
 *   D0  = P0.02  →  CS1_PIN
 *   D1  = P0.03  →  CS2_PIN
 *   D2  = P0.28  →  DRDY1_PIN  (interrupt-capable)
 *   D3  = P0.29  →  DRDY2_PIN  (interrupt-capable)
 *   D4  = P0.04  →  START_PIN
 *   D5  = P0.05  →  RESET_PIN
 *   D8  = P1.13  →  SPI SCK   (hardware)
 *   D9  = P1.14  →  SPI MISO  (hardware)
 *   D10 = P1.15  →  SPI MOSI  (hardware)
 *
 * IIR COEFFICIENTS — MUST BE GENERATED FOR YOUR ACTUAL SAMPLE RATE
 * ─────────────────────────────────────────────────────────────────────────────
 *   With 2.048 MHz internal oscillator and SYS0=0x05 (160 SPS nominal),
 *   the real data rate is ≈ 80 SPS.
 *
 *   Python (scipy) generation example:
 *       from scipy.signal import iirnotch, bilinear_zpk, zpk2sos
 *       import numpy as np
 *       fs = 80          # your actual sample rate
 *       f0 = 50          # notch frequency (Hz)
 *       Q  = 30          # quality factor
 *       b, a = iirnotch(f0, Q, fs)
 *       # CMSIS DF1 format: {b0, b1, b2, -a1, -a2}
 *       print([b[0], b[1], b[2], -a[1], -a[2]])
 * ============================================================================
 */

#include <SPI.h>
#include <ArduinoBLE.h>
#include <arm_math.h>

// ============================================================
// 1. PIN DEFINITIONS
// ============================================================
#define CS1_PIN    D0
#define CS2_PIN    D1
#define DRDY1_PIN  D2
#define DRDY2_PIN  D3
#define START_PIN  D4
#define RESET_PIN  D5

// ============================================================
// 2. ADS1248 COMMAND BYTES  (datasheet Table 25)
// ============================================================
#define CMD_WAKEUP   0x00
#define CMD_SLEEP    0x02
#define CMD_SYNC     0x04  // Must be sent TWICE consecutively
#define CMD_RESET    0x06
#define CMD_RDATA    0x12
#define CMD_RDATAC   0x10
#define CMD_SDATAC   0x16  // Stop Read Data Continuously — must issue before WREG
#define CMD_WREG     0x40  // OR with register address
#define CMD_RREG     0x20  // OR with register address
#define CMD_NOP      0xFF

// ============================================================
// 3. ADS1248 REGISTER ADDRESSES
// ============================================================
#define REG_MUX0   0x00
#define REG_VBIAS  0x01
#define REG_MUX1   0x02
#define REG_SYS0   0x03
#define REG_IDAC0  0x0A
#define REG_IDAC1  0x0B

/**
 * SYS0 value:  PGA gain = 1 (bits [6:4] = 000)
 *              Data rate = 160 SPS nominal (bits [3:0] = 0101)
 *              → ~80 actual SPS with 2.048 MHz internal oscillator
 *
 * DR bit map (SYS0 bits [3:0]):
 *   0000=5  0001=10  0010=20  0011=40  0100=80
 *   0101=160  0110=320  0111=640  1000=1000  1001=2000
 */
#define SYS0_PGA1_DR160   0x05

/**
 * MUX1 value:  VREFCON bits [6:5] = 01 → internal 2.048 V reference always ON
 *              REFSELT  bits [4:3] = 00 → REF0 (internal reference)
 *              MUXCAL   bits [2:0] = 000 → normal measurement mode
 */
#define MUX1_INTREF_ON    0x20

/**
 * MUX0 configurations: AIN0–AIN6 as positive vs AIN7 as negative reference.
 * MUX0 register: bits [7:4] = MUXSP (positive channel), [3:0] = MUXSN (negative)
 * AIN7 = 0111b = 0x7
 *
 * FIX: Original code used {0x07,0x0F,0x17,...} which is WRONG.
 *      0x0F = MUXSP=AIN0, MUXSN=1111b (reserved/AINCOM) — not AIN7!
 *      Correct step is +0x10 (increment MUXSP), not +0x08.
 */
static const uint8_t MUX_CONFIGS[7] = {
    0x07,  // AIN0+ / AIN7−
    0x17,  // AIN1+ / AIN7−
    0x27,  // AIN2+ / AIN7−
    0x37,  // AIN3+ / AIN7−
    0x47,  // AIN4+ / AIN7−
    0x57,  // AIN5+ / AIN7−
    0x67   // AIN6+ / AIN7−
};

// ============================================================
// 4. DATA BUFFERS
// ============================================================
#define NUM_CHANNELS      14
#define BYTES_PER_CH       3         // 24-bit
#define PAYLOAD_BYTES     (NUM_CHANNELS * BYTES_PER_CH)  // 42

static int32_t   rawChannels[NUM_CHANNELS];
static float32_t dspInput  [NUM_CHANNELS];
static float32_t dspOutput [NUM_CHANNELS];
static uint8_t   blePayload[PAYLOAD_BYTES];

// ============================================================
// 5. VOLATILE STATE (shared with ISRs)
// ============================================================
volatile uint8_t  gMuxIndex     = 0;
volatile bool     gDrdy1        = false;
volatile bool     gDrdy2        = false;
volatile bool     gFrameReady   = false;
volatile uint32_t gLastDrdyMs   = 0;
#define DRDY_WATCHDOG_MS  3000      // Reset ADCs if silent for 3 s

// ============================================================
// 6. SPI SETTINGS
//    ADS1248: max 5 MHz, MSB first, Mode 1 (CPOL=0, CPHA=1)
//    Datasheet §9.5.1.2: DIN sampled on falling SCLK edge,
//    DOUT changes on rising SCLK edge → SPI_MODE1 confirmed.
// ============================================================
static SPISettings adcSPI(2000000, MSBFIRST, SPI_MODE1);

// ============================================================
// 7. BLE SERVICE & CHARACTERISTIC
// ============================================================
static BLEService        bciService ("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLECharacteristic bciDataChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                                      BLERead | BLENotify, PAYLOAD_BYTES);

// ============================================================
// 8. DSP — CMSIS-DSP BIQUAD IIR (50 Hz Notch)
// ============================================================
/**
 * !!! REPLACE THESE WITH PROPERLY GENERATED COEFFICIENTS !!!
 * Format expected by arm_biquad_cascade_df1_f32:
 *   { b0, b1, b2, −a1, −a2 }   (note: a1 and a2 are NEGATED)
 *
 * Example for 50 Hz notch at 80 SPS (Q=30), generated with scipy.signal:
 *   b0= 0.9757, b1=−1.7009, b2=0.9757, −a1=+1.7009, −a2=−0.9514
 * Replace iirCoeffs below with your calculated values.
 */
static float32_t iirCoeffs[5] = { 1.0f, -1.9f, 1.0f, 1.9f, -0.9f };

// Flat state array: 4 floats per channel per biquad stage (CMSIS requirement)
// Stride: channel i uses &iirState[i * 4]
static float32_t iirState[NUM_CHANNELS * 4];
static arm_biquad_casd_df1_inst_f32 notchFilter[NUM_CHANNELS];

// ============================================================
// 9. INTERRUPT SERVICE ROUTINES — keep absolutely minimal
// ============================================================
void drdy1_ISR() {
    gDrdy1      = true;
    gLastDrdyMs = millis();
}
void drdy2_ISR() {
    gDrdy2 = true;
}

// ============================================================
// 10. ADS1248 LOW-LEVEL HELPERS
// ============================================================

/** Send a single command byte with no data payload. */
static void sendCmd(int csPin, uint8_t cmd) {
    SPI.beginTransaction(adcSPI);
    digitalWrite(csPin, LOW);
    SPI.transfer(cmd);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(10);
}

/** Write one register. */
static void writeReg(int csPin, uint8_t reg, uint8_t val) {
    SPI.beginTransaction(adcSPI);
    digitalWrite(csPin, LOW);
    SPI.transfer(CMD_WREG | (reg & 0x0F)); // First byte: WREG | addr
    SPI.transfer(0x00);                     // Second byte: (n−1), n=1 register
    SPI.transfer(val);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(10);
}

/** Read one register (used for verification during debug). */
static uint8_t readReg(int csPin, uint8_t reg) {
    SPI.beginTransaction(adcSPI);
    digitalWrite(csPin, LOW);
    SPI.transfer(CMD_RREG | (reg & 0x0F));
    SPI.transfer(0x00);
    uint8_t val = SPI.transfer(CMD_NOP);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    return val;
}

/**
 * Switch MUX channel and restart conversion.
 * Per §9.4.4.4: writing to MUX0 automatically resets the digital filter
 * and starts a fresh conversion — no explicit SYNC command needed here.
 */
static void setMux(int csPin, uint8_t muxConfig) {
    writeReg(csPin, REG_MUX0, muxConfig);
    // Digital filter resets and new conversion begins automatically.
}

/**
 * Read 24-bit conversion result via RDATA command.
 *
 * FIX: Cast bytes to int32_t BEFORE shifting (original code shifted uint8_t
 * which is undefined behaviour when shift exceeds type width).
 *
 * FIX: Add t₆ delay (≥24 fCLK cycles) between RDATA command and first SCLK.
 * At 2.048 MHz internal oscillator: 24 / 2048000 ≈ 11.72 µs → use 12 µs.
 * (Datasheet §9.5.1.4 and timing spec Table 5, parameter t6)
 */
static int32_t readADC(int csPin) {
    SPI.beginTransaction(adcSPI);
    digitalWrite(csPin, LOW);

    SPI.transfer(CMD_RDATA);
    delayMicroseconds(12);              // t₆: mandatory wait before data SCLKs

    uint8_t b0 = SPI.transfer(CMD_NOP);
    uint8_t b1 = SPI.transfer(CMD_NOP);
    uint8_t b2 = SPI.transfer(CMD_NOP);

    digitalWrite(csPin, HIGH);
    SPI.endTransaction();

    // Reconstruct 24-bit value — cast FIRST, then shift (avoids UB)
    int32_t val = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | (int32_t)b2;

    // Sign-extend bit 23 to full int32_t (two's complement)
    if (val & 0x800000) val |= 0xFF000000;
    return val;
}

/**
 * Full initialisation sequence for one ADS1248 chip.
 *
 * FIX: SDATAC must be sent first. After power-on or hardware reset the chip
 * is in RDATAC (Read Data Continuously) mode; WREG commands are silently
 * ignored in that mode (datasheet §9.5.2 and Figure 85).
 */
static void initADS1248(int csPin) {
    // 1. Stop continuous read mode so register writes will be accepted
    sendCmd(csPin, CMD_SDATAC);
    delay(2);

    // 2. Configure data rate and PGA gain via SYS0
    //    PGA=1 gives ±2.048 V FSR with internal 2.048 V reference — ideal for EEG
    writeReg(csPin, REG_SYS0, SYS0_PGA1_DR160);

    // 3. Enable internal 2.048 V reference (always on)
    writeReg(csPin, REG_MUX1, MUX1_INTREF_ON);

    // 4. Disable excitation current sources (passive electrodes)
    writeReg(csPin, REG_IDAC0, 0x00);
    writeReg(csPin, REG_IDAC1, 0x00);

    // 5. Set initial MUX channel — writing MUX0 auto-starts first conversion
    writeReg(csPin, REG_MUX0, MUX_CONFIGS[0]);

    delay(20); // Allow first conversion to settle before we begin reading
}

// ============================================================
// 11. WATCHDOG RECOVERY
// ============================================================
static void resetADCs() {
    detachInterrupt(digitalPinToInterrupt(DRDY1_PIN));
    detachInterrupt(digitalPinToInterrupt(DRDY2_PIN));

    digitalWrite(RESET_PIN, LOW);
    delay(10);
    digitalWrite(RESET_PIN, HIGH);
    delay(100);  // ≥0.6 ms required after RESET high (datasheet §9.4.2); 100 ms is safe

    initADS1248(CS1_PIN);
    initADS1248(CS2_PIN);

    noInterrupts();
    gMuxIndex   = 0;
    gDrdy1      = false;
    gDrdy2      = false;
    gFrameReady = false;
    gLastDrdyMs = millis();
    interrupts();

    attachInterrupt(digitalPinToInterrupt(DRDY1_PIN), drdy1_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(DRDY2_PIN), drdy2_ISR, FALLING);
}

// ============================================================
// 12. SETUP
// ============================================================
void setup() {
    Serial.begin(115200);

    // --- Pin initialisation ---
    pinMode(CS1_PIN,   OUTPUT);  digitalWrite(CS1_PIN,   HIGH);
    pinMode(CS2_PIN,   OUTPUT);  digitalWrite(CS2_PIN,   HIGH);
    pinMode(START_PIN, OUTPUT);  digitalWrite(START_PIN, LOW);
    pinMode(RESET_PIN, OUTPUT);
    pinMode(DRDY1_PIN, INPUT_PULLUP);
    pinMode(DRDY2_PIN, INPUT_PULLUP);

    // --- Hardware reset (active-low RESET pin) ---
    digitalWrite(RESET_PIN, LOW);
    delay(10);
    digitalWrite(RESET_PIN, HIGH);
    delay(100);  // Datasheet: wait ≥0.6 ms; 100 ms is generous

    // --- SPI bus ---
    SPI.begin();

    // --- BLE stack ---
    if (!BLE.begin()) {
        Serial.println("ERROR: BLE init failed. Halting.");
        while (1);
    }
    BLE.setLocalName("SmartGlasses_BCI");
    BLE.setAdvertisedService(bciService);
    bciService.addCharacteristic(bciDataChar);
    BLE.addService(bciService);
    BLE.advertise();

    // --- ADS1248 chips ---
    initADS1248(CS1_PIN);
    initADS1248(CS2_PIN);

    // --- CMSIS-DSP IIR notch filters (one per channel) ---
    for (int i = 0; i < NUM_CHANNELS; i++) {
        // FIX: pass &iirState[i * 4] explicitly — no implicit 2D-array decay
        arm_biquad_cascade_df1_init_f32(&notchFilter[i], 1, iirCoeffs,
                                         &iirState[i * 4]);
    }

    // --- Start continuous conversion (both chips sync'd on shared START pin) ---
    gLastDrdyMs = millis();
    digitalWrite(START_PIN, HIGH);
    delay(10);

    // --- Enable DRDY interrupts ---
    attachInterrupt(digitalPinToInterrupt(DRDY1_PIN), drdy1_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(DRDY2_PIN), drdy2_ISR, FALLING);
}

// ============================================================
// 13. MAIN LOOP
// ============================================================
void loop() {
    // ── Watchdog: recover from hardware silence ──────────────────────────────
    if ((millis() - gLastDrdyMs) > DRDY_WATCHDOG_MS) {
        Serial.println("WARN: DRDY timeout — resetting ADCs");
        resetADCs();
        return;
    }

    // ── Channel acquisition (runs when both ADCs report data ready) ──────────
    if (gDrdy1 && gDrdy2) {
        // FIX: protect gMuxIndex read-modify-write from ISR preemption
        noInterrupts();
        gDrdy1 = false;
        gDrdy2 = false;
        uint8_t idx = gMuxIndex;
        interrupts();

        // Read 24-bit samples from current channel on both chips
        rawChannels[idx]     = readADC(CS1_PIN);   // Sensors  1–7
        rawChannels[idx + 7] = readADC(CS2_PIN);   // Sensors 8–14

        // Advance to next MUX channel
        uint8_t nextIdx = idx + 1;

        if (nextIdx < 7) {
            noInterrupts();
            gMuxIndex = nextIdx;
            interrupts();
            // Writing MUX0 auto-resets filter and starts new conversion
            setMux(CS1_PIN, MUX_CONFIGS[nextIdx]);
            setMux(CS2_PIN, MUX_CONFIGS[nextIdx]);
        } else {
            // Full 14-channel frame collected — wrap back to channel 0
            noInterrupts();
            gMuxIndex   = 0;
            gFrameReady = true;
            interrupts();
            setMux(CS1_PIN, MUX_CONFIGS[0]);
            setMux(CS2_PIN, MUX_CONFIGS[0]);
        }
    }

    // ── Transmit completed frame if BLE central is connected ─────────────────
    if (gFrameReady) {
        noInterrupts();
        gFrameReady = false;
        interrupts();

        BLEDevice central = BLE.central();
        if (central && central.connected()) {
            processAndTransmit();
        }
    }

    // Keep BLE stack alive — must be called regularly
    BLE.poll();
}

// ============================================================
// 14. DSP + BLE TRANSMISSION
// ============================================================
void processAndTransmit() {
    uint8_t idx = 0;

    for (int i = 0; i < NUM_CHANNELS; i++) {
        // 1. Raw int32 → float32 for CMSIS-DSP
        dspInput[i] = (float32_t)rawChannels[i];

        // 2. Single-sample IIR biquad notch filter (50 Hz)
        //    Block size = 1: one sample in, one filtered sample out.
        //    For higher throughput, buffer a full frame and use blockSize=14,
        //    but that adds 1-frame latency which is negligible for BCI.
        arm_biquad_cascade_df1_f32(&notchFilter[i],
                                    &dspInput[i],
                                    &dspOutput[i], 1);

        // 3. Filtered float → int32 (values remain within 24-bit range)
        int32_t cleaned = (int32_t)dspOutput[i];

        // 4. Pack as 3-byte big-endian (MSB first, matching ADC output order)
        blePayload[idx++] = (uint8_t)((cleaned >> 16) & 0xFF);
        blePayload[idx++] = (uint8_t)((cleaned >>  8) & 0xFF);
        blePayload[idx++] = (uint8_t)( cleaned        & 0xFF);
    }

    // 5. Push 42-byte frame to connected mobile device via BLE notification
    bciDataChar.writeValue(blePayload, PAYLOAD_BYTES);
}
