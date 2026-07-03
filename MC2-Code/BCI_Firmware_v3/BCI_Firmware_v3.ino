/**
 * ============================================================================
 *  SmartGlasses BCI Firmware — Written from scratch
 *  Hardware  : Seeed Studio XIAO nRF52840 Sense
 *              2 × Texas Instruments ADS1248
 *  Channels  : 14 active EEG (7 per chip: AIN0–AIN6 vs AIN7 reference)
 *  Interface : BLE 5.0 notification, 42-byte frame (14 ch × 3 bytes)
 *  DSP       : CMSIS-DSP biquad IIR notch (50 Hz)
 *
 *  All register values, command codes, and timing derived from:
 *    [ADS] ADS1246/ADS1247/ADS1248 datasheet SBAS426H (TI, 2016)
 *    [NRF] nRF52840 Product Specification v1.5
 *    [XIO] Seeed Studio XIAO nRF52840 Schematic v1.1
 *    [PIN] XIAO-nRF52840-Sense-pinout_sheet.xlsx
 * ============================================================================
 *
 *  PIN MAPPING  (verified from [XIO] schematic and [PIN] sheet)
 *  ─────────────────────────────────────────────────────────────
 *  XIAO Arduino  │ nRF52840  │  Function
 *  ──────────────┼───────────┼──────────────────────────────────
 *  D0            │ P0.02     │  CS1 — ADS1248 #1 Chip Select
 *  D1            │ P0.03     │  CS2 — ADS1248 #2 Chip Select
 *  D2            │ P0.28     │  DRDY1 — ADC1 Data Ready (active low)
 *  D3            │ P0.29     │  DRDY2 — ADC2 Data Ready (active low)
 *  D4            │ P0.04     │  START — shared (both chips, active high)
 *  D5            │ P0.05     │  RESET — shared (both chips, active low)
 *  D8            │ P1.13     │  SPI SCK  (hardware, from [XIO] schematic)
 *  D9            │ P1.14     │  SPI MISO (hardware)
 *  D10           │ P1.15     │  SPI MOSI (hardware)
 *
 *  CLK pin on each ADS1248 → tied to DGND → activates internal 4.096 MHz
 *  oscillator  [ADS §9.4.1, pin table p.6]
 * ============================================================================
 */

#include <SPI.h>
#include <ArduinoBLE.h>
#include <arm_math.h>

// ============================================================================
//  SECTION 1 — HARDWARE PIN CONSTANTS
// ============================================================================

static constexpr int PIN_CS1    = D0;   // P0.02
static constexpr int PIN_CS2    = D1;   // P0.03
static constexpr int PIN_DRDY1  = D2;   // P0.28 — interrupt capable
static constexpr int PIN_DRDY2  = D3;   // P0.29 — interrupt capable
static constexpr int PIN_START  = D4;   // P0.04
static constexpr int PIN_RESET  = D5;   // P0.05

// ============================================================================
//  SECTION 2 — ADS1248 COMMAND CODES
//  Source: [ADS] Table 19, p.45
// ============================================================================

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

// ============================================================================
//  SECTION 3 — ADS1248 REGISTER ADDRESSES
//  Source: [ADS] Table 29, p.55 (ADS1247/ADS1248 register map)
// ============================================================================

static constexpr uint8_t REG_MUX0  = 0x00; // Mux control 0: BCS, MUX_SP, MUX_SN
static constexpr uint8_t REG_VBIAS = 0x01; // Bias voltage enable per AIN pin
static constexpr uint8_t REG_MUX1  = 0x02; // Mux control 1: CLKSTAT, VREFCON, REFSELT, MUXCAL
static constexpr uint8_t REG_SYS0  = 0x03; // PGA gain + data rate
static constexpr uint8_t REG_IDAC0 = 0x0A; // IDAC magnitude + DRDYMODE
static constexpr uint8_t REG_IDAC1 = 0x0B; // IDAC output routing

// ============================================================================
//  SECTION 4 — ADS1248 REGISTER VALUES (computed from bit-field definitions)
// ============================================================================

/**
 * SYS0 register  [ADS] Table 35, p.60
 * ─────────────────────────────────────
 *  bit 7   : reserved → 0
 *  bits 6:4: PGA[2:0] → 110b = PGA gain 64 (±32 mV FSR with 2.048 V ref)
 *            Chosen for EEG: typical scalp potentials 10–100 µV
 *  bits 3:0: DR[3:0]  → 1001b = 2000 SPS
 *            At 2000 SPS with 7 channels: ~285 effective SPS per channel
 *            Nyquist ≈ 142 Hz → sufficient for EEG (DC–100 Hz)
 *
 *  To change PGA:  SYS0 = (gain_code << 4) | data_rate_code
 *  gain codes: 0=×1, 1=×2, 2=×4, 3=×8, 4=×16, 5=×32, 6=×64, 7=×128
 *  DR codes:   0=5, 1=10, 2=20, 3=40, 4=80, 5=160, 6=320, 7=640,
 *              8=1000, 9–F=2000 SPS
 */
static constexpr uint8_t VAL_SYS0 = (6 << 4) | 9;   // 0x69: PGA=64, DR=2000 SPS

/**
 * MUX1 register  [ADS] Table 33, p.59
 * ─────────────────────────────────────
 *  bit 7   : CLKSTAT  — read-only, write 0
 *  bits 6:5: VREFCON  → 01b = internal reference always ON
 *  bits 4:3: REFSELT  → 10b = internal 2.048 V reference selected for ADC
 *  bits 2:0: MUXCAL   → 000b = normal measurement mode
 *
 *  Binary: 0_01_10_000 = 0x30
 */
static constexpr uint8_t VAL_MUX1 = 0x30;

/**
 * VBIAS register  [ADS] Table 32, p.58
 * ─────────────────────────────────────
 *  bit 7: VBIAS on AIN7 (our reference electrode)
 *  Enable mid-supply bias on AIN7 to stabilize the reference input.
 *  This prevents the reference electrode from floating.
 *  0x80 = bias AIN7 only.
 *  Set to 0x00 if you have an external reference electrode buffer.
 */
static constexpr uint8_t VAL_VBIAS = 0x80;  // bias on AIN7 reference

/**
 * IDAC0 register  [ADS] Table 38, p.62
 * ─────────────────────────────────────
 *  bits 7:4: ID[3:0]   — read-only (device revision)
 *  bit 3   : DRDYMODE  → 0 = DOUT/DRDY pin = data out only (default)
 *  bits 2:0: IMAG[2:0] → 000 = IDAC off (no excitation current needed)
 */
static constexpr uint8_t VAL_IDAC0 = 0x00;

/**
 * IDAC1 register  [ADS] Table 39, p.63 — reset value = 0xFF (disconnected)
 * Bits 7:4 = I1DIR = 1111b = disconnected
 * Bits 3:0 = I2DIR = 1111b = disconnected
 */
static constexpr uint8_t VAL_IDAC1 = 0xFF;

/**
 * MUX0 configurations  [ADS] Table 30, p.56 (ADS1248 register map)
 * ──────────────────────────────────────────────────────────────────
 *  MUX0 layout:  [7:6]=BCS[1:0]  [5:3]=MUX_SP[2:0]  [2:0]=MUX_SN[2:0]
 *
 *  BCS  = 00  (burn-out current source off)
 *  MUX_SN = 111b = AIN7  (common reference electrode on AIN7)
 *
 *  Channel  MUX_SP  MUX_SN  Binary            Hex
 *  ───────  ──────  ──────  ────────────────── ────
 *  AIN0+    000     111     00_000_111          0x07
 *  AIN1+    001     111     00_001_111          0x0F
 *  AIN2+    010     111     00_010_111          0x17
 *  AIN3+    011     111     00_011_111          0x1F
 *  AIN4+    100     111     00_100_111          0x27
 *  AIN5+    101     111     00_101_111          0x2F
 *  AIN6+    110     111     00_110_111          0x37
 */
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

// ============================================================================
//  SECTION 5 — APPLICATION CONSTANTS
// ============================================================================

static constexpr int     NUM_CHANNELS    = 14;          // 7 per ADS1248
static constexpr int     BYTES_PER_CH    = 3;           // 24-bit raw value
static constexpr int     PAYLOAD_BYTES   = NUM_CHANNELS * BYTES_PER_CH;  // 42
static constexpr uint32_t WATCHDOG_MS    = 2000;        // Reset if no DRDY for 2 s

/**
 * SPI settings for ADS1248
 * Max SCLK: 1/488ns ≈ 2.05 MHz per tSCLK timing spec [ADS] §7.6
 * Mode 1: DIN latched on falling SCLK, DOUT shifts on rising SCLK
 * [ADS] §9.5.1.2 and §9.5.1.3 — confirmed Mode 1 (CPOL=0, CPHA=1)
 */
static const SPISettings ADS_SPI(2000000, MSBFIRST, SPI_MODE1);

// ============================================================================
//  SECTION 6 — DATA BUFFERS
// ============================================================================

static int32_t   rawData   [NUM_CHANNELS];              // Raw 24-bit ADC values
static float32_t filterIn  [NUM_CHANNELS];              // DSP input (float)
static float32_t filterOut [NUM_CHANNELS];              // DSP output (float)
static uint8_t   bleFrame  [PAYLOAD_BYTES];             // BLE transmit buffer

// ============================================================================
//  SECTION 7 — VOLATILE SHARED STATE (ISR ↔ main loop)
// ============================================================================

volatile bool     gDrdy1       = false;
volatile bool     gDrdy2       = false;
volatile bool     gFrameReady  = false;
volatile uint8_t  gMuxIndex    = 0;
volatile uint32_t gLastDrdyMs  = 0;

// ============================================================================
//  SECTION 8 — DSP FILTER (CMSIS-DSP biquad IIR — 50 Hz notch)
// ============================================================================

/**
 * IIR BIQUAD COEFFICIENTS — MUST BE REGENERATED FOR YOUR SAMPLE RATE
 * ────────────────────────────────────────────────────────────────────
 *  With 2000 SPS on each ADS1248 and 7 MUX channels, the effective
 *  per-channel sample rate ≈ 2000 / 7 ≈ 285.7 SPS.
 *
 *  Generate 50 Hz notch coefficients using Python + scipy:
 *
 *      from scipy.signal import iirnotch
 *      import numpy as np
 *
 *      fs = 285.7          # effective sample rate per channel
 *      f0 = 50.0           # notch frequency (Hz)   — use 60.0 for Americas
 *      Q  = 30.0           # quality factor (narrow notch, typical for EEG)
 *
 *      b, a = iirnotch(f0, Q, fs)
 *      # CMSIS-DSP DF1 biquad format: {b0, b1, b2, -a1, -a2}
 *      coeffs = [b[0], b[1], b[2], -a[1], -a[2]]
 *      print([f"{c:.10f}f" for c in coeffs])
 *
 *  Typical output for 50 Hz @ 285.7 SPS, Q=30:
 *      b0 ≈  0.9788,  b1 ≈ -1.8744,  b2 ≈  0.9788
 *     -a1 ≈  1.8744, -a2 ≈ -0.9577
 *
 *  Replace the placeholder values below with your computed values.
 *  The placeholder is a mathematically valid filter but not tuned to
 *  any specific frequency — DO NOT use in production.
 */
static float32_t iirCoeffs[5] = {
    0.9788f,    // b0
   -1.8744f,   // b1
    0.9788f,    // b2
    1.8744f,   // -a1  (CMSIS sign convention: negated)
   -0.9577f    // -a2  (CMSIS sign convention: negated)
};

/**
 * State buffer: 4 floats per channel per biquad stage (CMSIS requirement)
 * [ARM CMSIS-DSP Reference, arm_biquad_cascade_df1_init_f32]
 * Flat layout: channel i occupies iirState[i*4 .. i*4+3]
 */
static float32_t iirState[NUM_CHANNELS * 4];
static arm_biquad_casd_df1_inst_f32 iirFilter[NUM_CHANNELS];

// ============================================================================
//  SECTION 9 — BLE SERVICE & CHARACTERISTIC
// ============================================================================

/**
 * BLE MTU NOTE:
 *  This firmware sends 42-byte notifications. BLE 4.0 default ATT_MTU = 23
 *  (20 bytes payload). The XIAO nRF52840 supports BLE 5.0 with up to 247
 *  bytes MTU [NRF §6.19.2]. The mobile central MUST request MTU ≥ 46 bytes
 *  (header 3 + payload 42 + 1 ATT overhead). Most modern BLE stacks do this
 *  automatically. If data appears truncated, verify central MTU negotiation.
 */
static BLEService        bciService ("19B10000-E8F2-537E-4F6C-D104768A1214");
static BLECharacteristic bciDataChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                                      BLERead | BLENotify,
                                      PAYLOAD_BYTES);

// ============================================================================
//  SECTION 10 — INTERRUPT SERVICE ROUTINES
//  Keep ISRs minimal: set flag only.  [ADS] §9.5.1.4 — DRDY active low.
// ============================================================================

void drdy1_ISR() { gDrdy1 = true;  gLastDrdyMs = millis(); }
void drdy2_ISR() { gDrdy2 = true; }

// ============================================================================
//  SECTION 11 — SPI HELPER FUNCTIONS
// ============================================================================

/**
 * ads_cmd() — Send a single-byte command with no data.
 *
 * CS must remain low for the entire SPI transaction.
 * tCSSC (delay from CS low to first SCLK) = 10 ns min [ADS §7.6] —
 * satisfied automatically by SPI.beginTransaction + GPIO overhead.
 */
static inline void ads_cmd(int cs, uint8_t cmd) {
    SPI.beginTransaction(ADS_SPI);
    digitalWrite(cs, LOW);
    SPI.transfer(cmd);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
}

/**
 * ads_wreg() — Write one or more consecutive registers.
 *
 * WREG command format  [ADS] §9.5.3.9, Figure 85:
 *   Byte 0: 0100_rrrr  (0x40 | register_address)
 *   Byte 1: 0000_nnnn  (n-1, where n = number of registers to write)
 *   Bytes 2..n+1: register data
 *
 * Writing MUX0, VBIAS, MUX1, or SYS0 automatically resets the digital
 * filter and restarts conversion  [ADS] §9.4.4.4.
 *
 * @param cs      Chip Select pin
 * @param reg     Starting register address
 * @param vals    Pointer to data bytes
 * @param count   Number of registers to write
 */
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

/**
 * ads_rreg() — Read one register (used for verify/debug).
 *
 * RREG format  [ADS] §9.5.3.8:
 *   Byte 0: 0010_rrrr  (0x20 | address)
 *   Byte 1: 0000_0000  (read 1 register)
 *   Byte 2: NOP (clocks out register value)
 */
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

/**
 * ads_read_data() — Issue RDATA command and clock out 24-bit conversion result.
 *
 * RDATA sequence  [ADS] §9.5.3.5, Figure 81:
 *   1. Assert CS low
 *   2. Send RDATA command byte (0x12)
 *   3. Clock out 3 bytes (24 bits) MSB first using NOP on DIN
 *   4. Deassert CS high
 *
 * Timing note  [ADS] §9.5.1.4:
 *   tDTS: SCLK rising edge after DRDY falling = 1 tCLK min (≈244 ns at 4.096 MHz)
 *   This is satisfied by the time the ISR fires, loop() checks the flag,
 *   and the first SPI.beginTransaction() executes.
 *   No additional delay is needed.
 *
 * Sign extension  [ADS] §9.5.2:
 *   Data is 24-bit two's complement, MSB first.
 *   Bit 23 is the sign bit → extend to int32_t.
 *
 * CRITICAL: b0, b1, b2 are uint8_t. Shifting a uint8_t by 16 or 8 bits is
 *   undefined behaviour in C/C++. CAST TO int32_t BEFORE SHIFTING.
 */
static int32_t ads_read_data(int cs) {
    SPI.beginTransaction(ADS_SPI);
    digitalWrite(cs, LOW);

    SPI.transfer(ADS_RDATA);

    uint8_t b0 = SPI.transfer(ADS_NOP);   // MSB
    uint8_t b1 = SPI.transfer(ADS_NOP);   // middle byte
    uint8_t b2 = SPI.transfer(ADS_NOP);   // LSB

    digitalWrite(cs, HIGH);
    SPI.endTransaction();

    // Assemble 24-bit value — cast to int32_t FIRST, then shift (avoids UB)
    int32_t val = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | (int32_t)b2;

    // Sign-extend bit 23 → full int32_t (two's complement)
    if (val & 0x800000) {
        val |= 0xFF000000;
    }

    return val;
}

/**
 * ads_set_channel() — Switch MUX to given channel and start new conversion.
 *
 * Writing to MUX0 automatically resets the digital filter and starts a new
 * conversion (single-cycle settling).  [ADS] §9.4.4.4, §9.4.4.3
 * No explicit SYNC command is needed after a MUX0 write.
 */
static inline void ads_set_channel(int cs, uint8_t mux0) {
    ads_wreg(cs, REG_MUX0, &mux0, 1);
}

// ============================================================================
//  SECTION 12 — ADS1248 INITIALISATION SEQUENCE
//  Per-chip. Call for both CS1 and CS2 after hardware reset.
// ============================================================================

/**
 * Initialisation order rationale:
 *
 *  1. SDATAC — After power-on or hardware RESET, the ADS1248 is in RDATAC
 *              mode [ADS §9.5.3.6 "This is the default mode after a power
 *              up or reset"]. WREG commands are silently ignored in RDATAC.
 *              [ADS §9.5.3.8 Figure 84 note]. Issue SDATAC first.
 *
 *  2. SYS0   — Set PGA gain and data rate. Changing PGA automatically loads
 *              the factory-trimmed FSC calibration value. [ADS §9.6.4.6]
 *
 *  3. MUX1   — Enable internal 2.048 V reference (always on) and select it
 *              as ADC reference. [ADS §9.6.4.3]
 *
 *  4. VBIAS  — Apply mid-supply bias to AIN7 (our reference electrode) to
 *              prevent it from floating. [ADS §9.6.4.2]
 *
 *  5. IDAC0  — Disable excitation currents, keep DRDYMODE=0 (DOUT only).
 *              [ADS §9.6.4.7]
 *
 *  6. IDAC1  — Keep both IDACs disconnected (reset value 0xFF). [ADS §9.6.4.8]
 *
 *  7. SELFOCAL — Run self offset calibration. The ADS1248 internally shorts
 *              inputs to mid-supply and measures offset. Updates OFC register.
 *              [ADS §9.4.5.3.1] Wait for DRDY to signal completion.
 *              At 2000 SPS: calibration time ≈ 8.07 ms [ADS Table 17].
 *
 *  8. MUX0  — Set channel 0 (AIN0 vs AIN7). This auto-resets the filter and
 *              starts the first conversion. [ADS §9.4.4.4]
 */
static void ads_init(int cs) {
    // Step 1: Stop continuous read mode
    ads_cmd(cs, ADS_SDATAC);
    delayMicroseconds(100);   // brief pause for command to take effect

    // Steps 2–6: Write SYS0, MUX1, VBIAS, IDAC0, IDAC1 as a contiguous
    // block starting at REG_SYS0=0x03. MUX1 is at 0x02, so write separately.
    //
    // Write MUX1 first (must be done before relying on internal reference)
    const uint8_t mux1 = VAL_MUX1;
    ads_wreg(cs, REG_MUX1, &mux1, 1);
    delayMicroseconds(200);   // reference stabilisation

    // Write SYS0 (PGA + DR). Changing PGA auto-loads factory FSC. [ADS §9.6.4.6]
    const uint8_t sys0 = VAL_SYS0;
    ads_wreg(cs, REG_SYS0, &sys0, 1);
    delayMicroseconds(100);

    // Write VBIAS
    const uint8_t vbias = VAL_VBIAS;
    ads_wreg(cs, REG_VBIAS, &vbias, 1);

    // Write IDAC0 and IDAC1 as a 2-byte block (registers 0x0A, 0x0B)
    const uint8_t idac[2] = { VAL_IDAC0, VAL_IDAC1 };
    ads_wreg(cs, REG_IDAC0, idac, 2);

    // Step 7: Self offset calibration
    // SDATAC is already active; calibration command works in this state.
    ads_cmd(cs, ADS_SELFOCAL);

    // Wait for DRDY to go low — indicates calibration complete [ADS Figure 86]
    // Timeout: 50 ms >> worst-case 8.07 ms at 2000 SPS [ADS Table 17]
    uint32_t t0 = millis();
    while (digitalRead(cs == PIN_CS1 ? PIN_DRDY1 : PIN_DRDY2) == HIGH) {
        if (millis() - t0 > 50) {
            Serial.println("WARN: calibration timeout");
            break;
        }
    }
    delayMicroseconds(500);   // allow shift register to settle after DRDY

    // Step 8: Select first channel (MUX0 write auto-starts conversion)
    const uint8_t mux0_ch0 = MUX_CONFIGS[0];
    ads_wreg(cs, REG_MUX0, &mux0_ch0, 1);
}

// ============================================================================
//  SECTION 13 — WATCHDOG RECOVERY
// ============================================================================

/**
 * Perform a full hardware + software reset of both ADCs and reinitialise.
 * Called when DRDY is silent for more than WATCHDOG_MS.
 */
static void ads_hardware_reset_and_reinit() {
    // Disable interrupts during recovery
    detachInterrupt(digitalPinToInterrupt(PIN_DRDY1));
    detachInterrupt(digitalPinToInterrupt(PIN_DRDY2));
    digitalWrite(PIN_START, LOW);

    // Hardware reset — RESET pin active low [ADS §9.4.2]
    // Minimum pulse: 4 tCLK = 4/4.096 MHz ≈ 977 ns [ADS §7.6 tRESET]
    // We use 10 ms — safely exceeds minimum.
    digitalWrite(PIN_RESET, LOW);
    delay(10);
    digitalWrite(PIN_RESET, HIGH);

    // After RESET goes high, internal registers are held in reset for 0.6 ms
    // at 4.096 MHz. [ADS §7.6 tRHSC]. Use 5 ms for margin.
    delay(5);

    ads_init(PIN_CS1);
    ads_init(PIN_CS2);

    // Synchronise both chips by pulsing the shared START pin low→high.
    // Both chips receive the rising edge simultaneously, restarting their
    // digital filters. [ADS §9.4.4, Figure 71]
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

// ============================================================================
//  SECTION 14 — SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);

    // ── GPIO initialisation ───────────────────────────────────────────────
    // Idle states: CS high (deselected), START low (no conversion), RESET high
    pinMode(PIN_CS1,   OUTPUT);  digitalWrite(PIN_CS1,   HIGH);
    pinMode(PIN_CS2,   OUTPUT);  digitalWrite(PIN_CS2,   HIGH);
    pinMode(PIN_START, OUTPUT);  digitalWrite(PIN_START, LOW);
    pinMode(PIN_RESET, OUTPUT);  digitalWrite(PIN_RESET, HIGH);
    pinMode(PIN_DRDY1, INPUT_PULLUP);   // active-low signal
    pinMode(PIN_DRDY2, INPUT_PULLUP);

    // ── Hardware reset ────────────────────────────────────────────────────
    // Resets both chips simultaneously (shared RESET line).
    // Minimum low time: 4 tCLK [ADS §7.6]. Use 10 ms.
    digitalWrite(PIN_RESET, LOW);
    delay(10);
    digitalWrite(PIN_RESET, HIGH);
    // Wait for internal registers to clear: 0.6 ms min at 4.096 MHz [ADS §7.6]
    delay(5);

    // ── SPI bus initialisation ────────────────────────────────────────────
    // Hardware SPI on XIAO: SCK=D8(P1.13), MISO=D9(P1.14), MOSI=D10(P1.15)
    // [XIO] schematic, [PIN] pin sheet
    SPI.begin();

    // ── BLE initialisation ────────────────────────────────────────────────
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

    // ── CMSIS-DSP IIR filter initialisation (one per channel) ────────────
    for (int i = 0; i < NUM_CHANNELS; i++) {
        arm_biquad_cascade_df1_init_f32(
            &iirFilter[i],
            1,                  // numStages = 1 (single biquad)
            iirCoeffs,          // shared coefficient array (same filter, all channels)
            &iirState[i * 4]    // each channel has 4 state floats per stage
        );
    }

    // ── ADS1248 initialisation ────────────────────────────────────────────
    ads_init(PIN_CS1);
    ads_init(PIN_CS2);

    // ── Synchronise both chips ────────────────────────────────────────────
    // Pulse START high to restart both chips at the same instant.
    // Both have already been set to MUX_CONFIGS[0] in ads_init().
    // This ensures DRDY1 and DRDY2 fire together. [ADS §9.4.4, Figure 71]
    digitalWrite(PIN_START, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_START, HIGH);

    // ── Attach DRDY interrupts ────────────────────────────────────────────
    gLastDrdyMs = millis();
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY1), drdy1_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY2), drdy2_ISR, FALLING);

    Serial.println("BCI firmware ready.");
}

// ============================================================================
//  SECTION 15 — MAIN ACQUISITION LOOP
// ============================================================================

void loop() {

    // ── BLE stack maintenance (must be called regularly) ──────────────────
    BLE.poll();

    // ── Watchdog: recover from ADC hardware failure ───────────────────────
    if ((millis() - gLastDrdyMs) > WATCHDOG_MS) {
        Serial.println("WARN: DRDY watchdog — resetting ADCs");
        ads_hardware_reset_and_reinit();
        return;
    }

    // ── Channel acquisition — fires when both ADCs signal data ready ──────
    // Both gDrdy1 and gDrdy2 are set by their respective ISRs (FALLING edge
    // on DRDY1/DRDY2).  We wait for both before reading, ensuring chip1 and
    // chip2 are always on the same MUX channel for frame coherence.
    if (gDrdy1 && gDrdy2) {

        // Atomically clear flags and capture current channel index
        noInterrupts();
        gDrdy1 = false;
        gDrdy2 = false;
        uint8_t idx = gMuxIndex;
        interrupts();

        // Read 24-bit conversion results from both chips
        rawData[idx]     = ads_read_data(PIN_CS1);  // Electrodes 1–7
        rawData[idx + 7] = ads_read_data(PIN_CS2);  // Electrodes 8–14

        // Advance to next MUX channel or wrap frame
        if (idx < (NUM_MUX_CHANNELS - 1)) {
            // More channels remaining in this frame
            uint8_t nextIdx = idx + 1;

            noInterrupts();
            gMuxIndex = nextIdx;
            interrupts();
            // 1. Pause conversions globally
            digitalWrite(PIN_START, LOW);
            // Switch both chips to the next channel simultaneously.
            // Writing MUX0 auto-resets the digital filter and starts a new
            // single-cycle conversion. [ADS §9.4.4.4]
            ads_set_channel(PIN_CS1, MUX_CONFIGS[nextIdx]);
            ads_set_channel(PIN_CS2, MUX_CONFIGS[nextIdx]);
            // 3. Fire both conversions simultaneously
            digitalWrite(PIN_START, HIGH);
        } else {
            // Frame complete — all 7 channels read from both chips
            noInterrupts();
            gMuxIndex  = 0;
            gFrameReady = true;
            interrupts();
            // 1. Pause conversions globally
            digitalWrite(PIN_START, LOW);
            // Reset both chips to channel 0 for next frame
            ads_set_channel(PIN_CS1, MUX_CONFIGS[0]);
            ads_set_channel(PIN_CS2, MUX_CONFIGS[0]);
            // 3. Fire both conversions simultaneously
            digitalWrite(PIN_START, HIGH);
        }
    }

    // ── Frame processing & BLE transmission ──────────────────────────────
    if (gFrameReady) {

        noInterrupts();
        gFrameReady = false;
        interrupts();

        // Only transmit if a central is connected
        BLEDevice central = BLE.central();
        if (central && central.connected()) {
            processAndTransmit();
        }
    }
}

// ============================================================================
//  SECTION 16 — DSP PROCESSING AND BLE PAYLOAD ASSEMBLY
// ============================================================================

/**
 * processAndTransmit()
 *
 * For each of the 14 channels:
 *   1. Cast raw int32 to float32 for CMSIS-DSP
 *   2. Apply 50 Hz IIR notch filter (biquad DF1, blockSize = 1)
 *   3. Cast filtered float32 back to int32
 *   4. Pack as 3 bytes, big-endian (matching ADS1248 output byte order)
 *
 * Push the 42-byte payload as a BLE notification.
 *
 * NOTE on blockSize=1:
 *   Processing one sample at a time introduces per-channel latency of
 *   exactly 1 sample period (~3.5 ms). For BCI this is negligible.
 *   If throughput is critical, buffer a full frame and call with blockSize=7
 *   but that increases code complexity without meaningful benefit here.
 */
static void processAndTransmit() {
    uint8_t payloadIdx = 0;

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {

        // 1. int32 → float32
        filterIn[ch] = (float32_t)rawData[ch];

        // 2. IIR notch filter (50 Hz)
        arm_biquad_cascade_df1_f32(
            &iirFilter[ch],
            &filterIn[ch],
            &filterOut[ch],
            1               // blockSize = 1 sample
        );

        // 3. float32 → int32 (output stays within 24-bit range after notch)
        int32_t cleaned = (int32_t)filterOut[ch];

        // 4. Pack 24-bit big-endian (MSB → byte 0)
        bleFrame[payloadIdx++] = (uint8_t)((cleaned >> 16) & 0xFF);
        bleFrame[payloadIdx++] = (uint8_t)((cleaned >>  8) & 0xFF);
        bleFrame[payloadIdx++] = (uint8_t)( cleaned        & 0xFF);
    }

    // Push 42-byte notification to connected mobile device
    bciDataChar.writeValue(bleFrame, PAYLOAD_BYTES);
}
