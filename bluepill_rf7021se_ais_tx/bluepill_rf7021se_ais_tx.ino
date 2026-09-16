// Debug variant: omit per-AIS-transmission RF calculation and STATUS logs.
// Startup, configuration, and manual diagnostics remain enabled.
// Variant: OLED startup: wait 800 ms; I2C 400 kHz; one display update per processed AIS sentence, after TX or on error.
/*
  Bluepill + RF7021SE / ADF7021 AIS transmitter from USB AIVDM
  Quiet debug version: no periodic STATUS spam; PB13 LED linked to PAC/PA-TX

  Purpose:
    - USB CDC Serial receives a single-fragment !AIVDM / !AIVDO NMEA sentence.
    - Firmware converts the AIS 6-bit payload to an AIS-like over-air HDLC/NRZI bitstream.
    - RF7021SE/ADF7021 transmits on 161.975 MHz using UART/SPI mode:
        ADF7021 TxRxCLK / module DCLK = transmit DATA input.
        ADF7021 TxRxDATA / module DATA = high-Z in transmit UART/SPI mode.
    - PB13 LED is ON whenever PAC/PA-TX is active. PB12 is left as input.
    - Serial1 is debug/command UART only.
    - 0.91-inch 128x32 I2C OLED displays the received AIVDM sentence and TX status.
    This build targets the Roger Clark Arduino_STM32 core and remaps I2C1 to PB8/PB9.
    - ADF7021 internal PN9 test pattern mode is available from the debug UART.

  IMPORTANT:
    161.975 MHz is AIS Channel A. Use ONLY into dummy load / attenuator /
    shielded box / direct-coupled test receiver. Do not radiate.

  Bluepill wiring used in this project:
    RF7021SE DATA    -> PA0   // TxRxDATA, high-Z in TX UART/SPI mode
    RF7021SE SDATA   -> PA1
    RF7021SE CE      -> PA2
    RF7021SE PAC     -> PA3
    RF7021SE SLE     -> PA4
    RF7021SE SREAD   -> PA5
    RF7021SE SCLK    -> PA6
    RF7021SE INT/LK  -> PA7
    RF7021SE DCLK    -> PB0   // TxRxCLK, transmit DATA input in UART/SPI mode
    RF7021SE MUX     -> PB1   // set to digital lock detect through R0 DB31:DB29 = 010

    TX LED           -> PB13  // active-high in this sketch
    PB12             -> input only / unused
    OLED SCL          -> PB8
    OLED SDA          -> PB9

  USB CDC Serial:
    Raw NMEA input, for example:
       

  Debug UART:
    Serial1 TX PA9, RX PA10, 115200 bps
    PA10 internal pull-up is enabled so the board does not misbehave when USB-UART is removed.
    STATUS is printed only on explicit command or key state changes, not periodically.

  RF setup:
    REF = 19.68 MHz
    AIS CH A = 161.975 MHz
    External VCO + RF_DIV2, R counter = 2
    Default VCO_BIAS=2, VCO_ADJUST=2 based on bench lock results.

  Notes:
    - AIS standard GMSK uses BT=0.4. ADF7021 Gaussian 2FSK/GMSK-like mode is BT=0.5.
      This is intended as a practical simulator/test source, not a certified AIS transmitter.
    - If an AIS receiver does not decode, try debug command 'i' to invert TX data polarity.
*/

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

// -----------------------------------------------------------------------------
// Serial ports
// -----------------------------------------------------------------------------
#define USB_NMEA Serial
#define DBG      Serial1

static const uint32_t USB_BAUD = 115200;   // USB CDC baud value is mostly symbolic, but useful on some cores.
static const uint32_t DBG_BAUD = 115200;

// -----------------------------------------------------------------------------
// Pins
// -----------------------------------------------------------------------------
#define PIN_ADF_DATA   PA0
#define PIN_ADF_SDATA  PA1
#define PIN_ADF_CE     PA2
#define PIN_ADF_PAC    PA3
#define PIN_ADF_SLE    PA4
#define PIN_ADF_SREAD  PA5
#define PIN_ADF_SCLK   PA6
#define PIN_ADF_INTLK  PA7
#define PIN_ADF_DCLK   PB0
#define PIN_ADF_MUX    PB1

#define PIN_UART_TX    PA9
#define PIN_UART_RX    PA10
#define PIN_TX_LED     PB13
#define PIN_PB12_UNUSED PB12

// 0.91-inch 128x32 I2C OLED (usually SSD1306, address 0x3C)
#define PIN_OLED_SCL    PB8
#define PIN_OLED_SDA    PB9
#define OLED_ADDR       0x3C
#define OLED_WIDTH      128
#define OLED_HEIGHT     32

// Roger Clark Arduino_STM32 core (STM32F1):
// I2C1 remap selects PB8=SCL and PB9=SDA.
TwoWire OLEDWire(1, I2C_REMAP | I2C_FAST_MODE); // 400 kHz
static bool g_oledOk = false;
static uint8_t g_oledBuf[OLED_WIDTH * OLED_HEIGHT / 8];

#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN INPUT
#endif

// -----------------------------------------------------------------------------
// RF / ADF7021 settings
// -----------------------------------------------------------------------------
static const uint32_t REF_HZ = 19680000UL;
static const uint32_t FRAC_DEN = 32768UL;
static const uint32_t RF_CH_A = 161975000UL;
static const uint32_t RF_CH_B = 162025000UL;

static uint32_t g_rfHz = RF_CH_A;
static uint8_t  g_rCounter = 2;     // 19.68 MHz / 2 = 9.84 MHz PFD; keeps INT_N >= 23 at 162 MHz.
static uint8_t  g_paLevel = 4;      // keep low for bench testing; increase only if really needed.
static uint8_t  g_vcoBias = 2;      // recommended for external VCO, RF_DIV2, 80-200 MHz.
static uint8_t  g_vcoAdjust = 2;    // bench result: 2/3 locked; default 2.
static bool     g_invertTxData = false;
static bool     g_strictChecksum = false; // default false because pasted/test AIVDM lines often have stale checksums.

// -----------------------------------------------------------------------------
// AIS frame buffers
// -----------------------------------------------------------------------------
static const uint16_t MAX_NMEA_LINE   = 128;
static const uint16_t MAX_PAYLOAD_BITS = 600;
static const uint16_t MAX_FRAME_BITS   = 1200;

static char g_usbLine[MAX_NMEA_LINE];
static uint16_t g_usbLineLen = 0;

static uint8_t g_payloadBits[MAX_PAYLOAD_BITS];
static uint8_t g_frameBits[MAX_FRAME_BITS];
static uint16_t g_payloadBitLen = 0;
static uint16_t g_frameBitLen = 0;

static bool g_busy = false;
static bool g_paActive = false;
static bool g_txLedRequested = false;

enum RadioMode : uint8_t {
  MODE_OFF,
  MODE_AIS_BURST,
  MODE_PN9,
  MODE_CARRIER
};
static RadioMode g_mode = MODE_OFF;


// -----------------------------------------------------------------------------
// Minimal SSD1306 driver for the old Roger Clark Arduino_STM32 Wire library.
// No Adafruit_GFX / Adafruit_SSD1306 / Adafruit_BusIO dependency.
// -----------------------------------------------------------------------------
static const uint8_t kFont5x7[96][5] = {
{0,0,0,0,0},{0,0,95,0,0},{0,7,0,7,0},{20,127,20,127,20},{36,42,127,42,18},{35,19,8,100,98},{54,73,85,34,80},{0,5,3,0,0},{0,28,34,65,0},{0,65,34,28,0},{20,8,62,8,20},{8,8,62,8,8},{0,80,48,0,0},{8,8,8,8,8},{0,96,96,0,0},{32,16,8,4,2},
{62,81,73,69,62},{0,66,127,64,0},{66,97,81,73,70},{33,65,69,75,49},{24,20,18,127,16},{39,69,69,69,57},{60,74,73,73,48},{1,113,9,5,3},{54,73,73,73,54},{6,73,73,41,30},{0,54,54,0,0},{0,86,54,0,0},{8,20,34,65,0},{20,20,20,20,20},{0,65,34,20,8},{2,1,81,9,6},
{50,73,121,65,62},{126,17,17,17,126},{127,73,73,73,54},{62,65,65,65,34},{127,65,65,34,28},{127,73,73,73,65},{127,9,9,9,1},{62,65,73,73,122},{127,8,8,8,127},{0,65,127,65,0},{32,64,65,63,1},{127,8,20,34,65},{127,64,64,64,64},{127,2,12,2,127},{127,4,8,16,127},{62,65,65,65,62},
{127,9,9,9,6},{62,65,81,33,94},{127,9,25,41,70},{70,73,73,73,49},{1,1,127,1,1},{63,64,64,64,63},{31,32,64,32,31},{63,64,56,64,63},{99,20,8,20,99},{3,4,120,4,3},{97,81,73,69,67},{0,127,65,65,0},{2,4,8,16,32},{0,65,65,127,0},{4,2,1,2,4},{64,64,64,64,64},{0,1,2,4,0},
{32,84,84,84,120},{127,72,68,68,56},{56,68,68,68,32},{56,68,68,72,127},{56,84,84,84,24},{8,126,9,1,2},{12,82,82,82,62},{127,8,4,4,120},{0,68,125,64,0},{32,64,68,61,0},{127,16,40,68,0},{0,65,127,64,0},{124,4,24,4,120},{124,8,4,4,120},{56,68,68,68,56},{124,20,20,20,8},
{8,20,20,24,124},{124,8,4,4,8},{72,84,84,84,32},{4,63,68,64,32},{60,64,64,32,124},{28,32,64,32,28},{60,64,48,64,60},{68,40,16,40,68},{12,80,80,80,60},{68,100,84,76,68},{0,8,54,65,0},{0,0,127,0,0},{0,65,54,8,0},{2,1,2,4,2},{0,0,0,0,0}
};

static bool oledWriteCommand(uint8_t cmd) {
  OLEDWire.beginTransmission(OLED_ADDR);
  OLEDWire.write((uint8_t)0x00);
  OLEDWire.write(cmd);
  return OLEDWire.endTransmission() == 0;
}

static void oledClearBuffer() {
  memset(g_oledBuf, 0, sizeof(g_oledBuf));
}

static void oledDrawChar(uint8_t x, uint8_t page, char c) {
  if (page >= 4 || x >= OLED_WIDTH) return;
  if (c < 32 || c > 127) c = '?';
  uint8_t idx = (uint8_t)c - 32;
  uint16_t base = (uint16_t)page * OLED_WIDTH + x;
  for (uint8_t i = 0; i < 5 && (x + i) < OLED_WIDTH; i++) {
    g_oledBuf[base + i] = kFont5x7[idx][i];
  }
  if (x + 5 < OLED_WIDTH) g_oledBuf[base + 5] = 0;
}

static void oledDrawText(uint8_t x, uint8_t page, const char *text) {
  while (*text && x <= 122) {
    oledDrawChar(x, page, *text++);
    x += 6;
  }
}

static bool oledFlush() {
  for (uint8_t page = 0; page < 4; page++) {
    oledWriteCommand((uint8_t)(0xB0 | page));
    oledWriteCommand(0x00);
    oledWriteCommand(0x10);
    for (uint8_t chunk = 0; chunk < 8; chunk++) {
      OLEDWire.beginTransmission(OLED_ADDR);
      OLEDWire.write((uint8_t)0x40);
      uint16_t offset = (uint16_t)page * OLED_WIDTH + (uint16_t)chunk * 16;
      OLEDWire.write(&g_oledBuf[offset], 16);
      if (OLEDWire.endTransmission() != 0) return false;
    }
  }
  return true;
}

static bool oledBegin() {
  delay(20);
  static uint8_t initSeq[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40,
    0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x02,
    0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
    0xAF
  };
  for (uint8_t i = 0; i < sizeof(initSeq); i++) {
    if (!oledWriteCommand(initSeq[i])) return false;
  }
  oledClearBuffer();
  return oledFlush();
}

static void oledShowBoot() {
  if (!g_oledOk) return;
  oledClearBuffer();
  oledDrawText(0, 0, "AIS RF Simulator");
  oledDrawText(0, 1, "161.975 MHz");
  oledDrawText(0, 2, "USB AIVDM ready");
  oledDrawText(0, 3, "TX: idle");
  oledFlush();
}

static void oledShowSentence(const char *line, const char *statusText) {
  if (!g_oledOk) return;
  oledClearBuffer();
  size_t pos = 0;
  size_t len = strlen(line);
  char row[22];
  for (uint8_t page = 0; page < 3; page++) {
    uint8_t n = 0;
    while (n < 21 && pos < len) row[n++] = line[pos++];
    row[n] = 0;
    oledDrawText(0, page, row);
  }
  oledDrawText(0, 3, statusText);
  oledFlush();
}

static void oledShowStatusOnly(const char *statusText) {
  if (!g_oledOk) return;
  memset(&g_oledBuf[3 * OLED_WIDTH], 0, OLED_WIDTH);
  oledDrawText(0, 3, statusText);
  oledFlush();
}

// -----------------------------------------------------------------------------
// Bit-buffer helpers
// -----------------------------------------------------------------------------
static bool appendBit(uint8_t *bits, uint16_t &n, uint16_t maxBits, uint8_t b) {
  if (n >= maxBits) return false;
  bits[n++] = b ? 1 : 0;
  return true;
}

static bool appendFlag(uint8_t *bits, uint16_t &n, uint16_t maxBits) {
  // HDLC flag 0x7E. This bit pattern is symmetric when reversed.
  static const uint8_t flag[8] = {0,1,1,1,1,1,1,0};
  for (uint8_t i = 0; i < 8; i++) {
    if (!appendBit(bits, n, maxBits, flag[i])) return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
// GPIO / ADF serial interface
// -----------------------------------------------------------------------------
// PB13 LED control.
// Default is active-high: PB13 -> resistor -> LED -> GND.
// If your LED is wired to 3.3 V and sinks into PB13, change this to 0.
#define TX_LED_ACTIVE_HIGH 1

static void forceTxLedOutput(bool physicalHigh) {
  // PB13 is confirmed usable on the current proto board; use normal Arduino GPIO only.
  pinMode(PIN_TX_LED, OUTPUT);
  digitalWrite(PIN_TX_LED, physicalHigh ? HIGH : LOW);
}

static void setTxLed(bool on) {
  g_txLedRequested = on;
#if TX_LED_ACTIVE_HIGH
  forceTxLedOutput(on);
#else
  forceTxLedOutput(!on);
#endif
}

// Keep the TX LED strictly linked to the module PAC/PA-TX control.
// LED ON means PAC is asserted and the RF output path is intended to be active.
static void setPaActive(bool on) {
  g_paActive = on;
  digitalWrite(PIN_ADF_PAC, on ? HIGH : LOW);
  setTxLed(on);
}

static void ledSelfTest() {
  DBG.println("PB13 LED self-test: ON 500 ms, OFF");
  setTxLed(true);
  delay(500);
  setTxLed(false);
}

static void idleBus() {
  digitalWrite(PIN_ADF_SLE, LOW);
  digitalWrite(PIN_ADF_SCLK, LOW);
  digitalWrite(PIN_ADF_SDATA, LOW);
}

static void dclkAsOutputLow() {
  pinMode(PIN_ADF_DCLK, OUTPUT);
  digitalWrite(PIN_ADF_DCLK, LOW);
}

static void dclkAsInputPulldown() {
  pinMode(PIN_ADF_DCLK, INPUT_PULLDOWN);
}

static void dataPinHiZ() {
  // TxRxDATA is high-Z in TX UART/SPI mode; keep MCU side as input too.
  pinMode(PIN_ADF_DATA, INPUT_PULLDOWN);
}

static void adfWriteReg(uint32_t word) {
  noInterrupts();
  digitalWrite(PIN_ADF_SLE, LOW);
  digitalWrite(PIN_ADF_SCLK, LOW);
  delayMicroseconds(2);

  for (uint8_t i = 0; i < 32; i++) {
    digitalWrite(PIN_ADF_SDATA, (word & 0x80000000UL) ? HIGH : LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_ADF_SCLK, HIGH);
    delayMicroseconds(1);
    word <<= 1;
    digitalWrite(PIN_ADF_SCLK, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(PIN_ADF_SLE, HIGH);
  delayMicroseconds(2);
  digitalWrite(PIN_ADF_SDATA, LOW);
  digitalWrite(PIN_ADF_SLE, LOW);
  delayMicroseconds(5);
  interrupts();
}

static uint16_t adfReadback(uint8_t cfg) {
  uint32_t cmd = ((uint32_t)(cfg & 0x1F) << 4) | 0x7UL;

  noInterrupts();
  digitalWrite(PIN_ADF_SLE, LOW);
  digitalWrite(PIN_ADF_SCLK, LOW);
  delayMicroseconds(2);

  for (uint8_t i = 0; i < 32; i++) {
    digitalWrite(PIN_ADF_SDATA, (cmd & 0x80000000UL) ? HIGH : LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_ADF_SCLK, HIGH);
    delayMicroseconds(1);
    cmd <<= 1;
    digitalWrite(PIN_ADF_SCLK, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(PIN_ADF_SLE, HIGH);
  digitalWrite(PIN_ADF_SDATA, LOW);

  // Dummy bit, per module example code.
  digitalWrite(PIN_ADF_SCLK, HIGH); delayMicroseconds(1);
  digitalWrite(PIN_ADF_SCLK, LOW);  delayMicroseconds(1);

  uint16_t rx = 0;
  for (uint8_t i = 0; i < 16; i++) {
    digitalWrite(PIN_ADF_SCLK, HIGH); delayMicroseconds(1);
    rx <<= 1;
    digitalWrite(PIN_ADF_SCLK, LOW);  delayMicroseconds(1);
    if (digitalRead(PIN_ADF_SREAD)) rx |= 1;
  }

  digitalWrite(PIN_ADF_SCLK, HIGH); delayMicroseconds(1);
  digitalWrite(PIN_ADF_SCLK, LOW);
  digitalWrite(PIN_ADF_SLE, LOW);
  delayMicroseconds(5);
  interrupts();

  return rx;
}

static void ceReset() {
  setPaActive(false);
  dclkAsOutputLow();
  dataPinHiZ();
  idleBus();
  digitalWrite(PIN_ADF_CE, LOW);
  delay(20);
  digitalWrite(PIN_ADF_CE, HIGH);
  delay(15);
}

// -----------------------------------------------------------------------------
// ADF7021 register builders, based on known-good 19.68 MHz settings
// -----------------------------------------------------------------------------
static double pfdHz() {
  return (double)REF_HZ / (double)g_rCounter;
}

static void calcN(uint32_t rfHz, uint16_t &intN, uint16_t &fracN) {
  double denom = pfdHz() * 0.5;  // RF_DIV2 enabled.
  double n = (double)rfHz / denom;
  intN = (uint16_t)floor(n);
  uint32_t frac = (uint32_t)((n - (double)intN) * (double)FRAC_DEN + 0.5);
  if (frac >= FRAC_DEN) {
    intN++;
    frac -= FRAC_DEN;
  }
  fracN = (uint16_t)frac;
}

static uint32_t buildR0(uint32_t rfHz) {
  uint16_t intN, fracN;
  calcN(rfHz, intN, fracN);

  uint32_t r0 = 0;
  r0 |= ((uint32_t)(fracN & 0x7FFF) << 4);
  r0 |= ((uint32_t)(intN  & 0x00FF) << 19);

  // DB31:DB29 = 010 -> MUXOUT digital lock detect
  // DB28 = 1        -> UART/SPI mode enabled
  // DB27 = 0        -> TX mode
  r0 |= (0b01010UL << 27);
  return r0;
}

static uint32_t buildR1_external_vco_div2() {
  uint32_t r1 = 0;
  r1 |= 1UL;                                      // address bits = 1
  r1 |= ((uint32_t)(g_rCounter & 0x7) << 4);      // R counter = 2
  r1 |= (1UL << 12);                              // XOSC enable; known-good for this module
  r1 |= (3UL << 13);                              // XTAL bias
  r1 |= (3UL << 15);                              // charge pump current
  r1 |= (1UL << 17);                              // VCO enable
  r1 |= (1UL << 18);                              // RF_DIVIDE_BY_2
  r1 |= ((uint32_t)(g_vcoBias & 0x0F) << 19);     // VCO_BIAS
  r1 |= ((uint32_t)(g_vcoAdjust & 0x03) << 23);   // VCO_ADJUST
  r1 |= (1UL << 25);                              // external inductor VCO
  return r1;
}

static uint32_t buildR3_9600() {
  // Known-good value from earlier tests with 19.68 MHz reference.
  uint32_t r3 = 0;
  r3 |= 3UL;
  r3 |= (2UL << 4);       // BBOS clock divide
  r3 |= (4UL << 6);       // DEMOD clock divide
  r3 |= (16UL << 10);     // CDR clock divide
  r3 |= (197UL << 18);    // sequencer clock divide
  r3 |= (10UL << 26);     // AGC clock divide
  return r3;
}

static uint32_t buildR2_tx(bool paEnable) {
  uint32_t r2 = 0;
  r2 |= 2UL;
  r2 |= (1UL << 4);                         // GFSK/GMSK-like Gaussian 2FSK
  if (paEnable) r2 |= (1UL << 7);            // PA enable
  r2 |= (7UL << 8);                          // PA ramp
  r2 |= (3UL << 11);                         // PA bias
  r2 |= ((uint32_t)(paEnable ? g_paLevel : 0) << 13);
  r2 |= (32UL << 19);                        // fdev ~= 2.4 kHz with PFD=9.84 MHz, RF_DIV2
  return r2;
}

static uint32_t buildR15_normal() {
  return 15UL;                               // no test pattern, UART-style data input on TxRxCLK
}

static uint32_t buildR15_tx_test(uint8_t mode) {
  return 15UL | ((uint32_t)(mode & 0x07) << 8);  // 1 carrier, 4 1010, 5 PN9, etc.
}

// -----------------------------------------------------------------------------
// Debug printing
// -----------------------------------------------------------------------------
static const char *modeName() {
  switch (g_mode) {
    case MODE_OFF: return "OFF";
    case MODE_AIS_BURST: return "AIS_BURST";
    case MODE_PN9: return "PN9";
    case MODE_CARRIER: return "CARRIER";
  }
  return "?";
}

static void printN() {
  uint16_t intN, fracN;
  calcN(g_rfHz, intN, fracN);
  double denom = pfdHz() * 0.5;
  double actual = denom * ((double)intN + (double)fracN / (double)FRAC_DEN);

  DBG.print("RF="); DBG.print(g_rfHz);
  DBG.print(" REF="); DBG.print(REF_HZ);
  DBG.print(" R="); DBG.print(g_rCounter);
  DBG.print(" PFD="); DBG.print(pfdHz(), 1);
  DBG.print(" INT="); DBG.print(intN);
  DBG.print(" FRAC="); DBG.print(fracN);
  DBG.print(" actual="); DBG.print(actual, 1);
  DBG.print(" errHz="); DBG.println(actual - (double)g_rfHz, 1);
}

static void printWords() {
  DBG.println("Words:");
  DBG.print("  R1_extVCO=0x"); DBG.println(buildR1_external_vco_div2(), HEX);
  DBG.print("  R15_NORMAL=0x"); DBG.println(buildR15_normal(), HEX);
  DBG.print("  R15_CARRIER=0x"); DBG.println(buildR15_tx_test(1), HEX);
  DBG.print("  R15_PN9=0x"); DBG.println(buildR15_tx_test(5), HEX);
  DBG.print("  R3=0x"); DBG.println(buildR3_9600(), HEX);
  DBG.print("  R0=0x"); DBG.println(buildR0(g_rfHz), HEX);
  DBG.print("  R2_ON=0x"); DBG.println(buildR2_tx(true), HEX);
  DBG.print("  R2_OFF=0x"); DBG.println(buildR2_tx(false), HEX);
}

static void status() {
  DBG.print("STATUS mode="); DBG.print(modeName());
  DBG.print(" busy="); DBG.print(g_busy ? 1 : 0);
  DBG.print(" RF="); DBG.print(g_rfHz);
  DBG.print(" PAlevel="); DBG.print(g_paLevel);
  DBG.print(" VCOBIAS="); DBG.print(g_vcoBias);
  DBG.print(" VCOADJ="); DBG.print(g_vcoAdjust);
  DBG.print(" invert="); DBG.print(g_invertTxData ? 1 : 0);
  DBG.print(" strictCS="); DBG.print(g_strictChecksum ? 1 : 0);
  DBG.print(" PAC="); DBG.print(digitalRead(PIN_ADF_PAC));
  DBG.print(" PAreq="); DBG.print(g_paActive ? 1 : 0);
  DBG.print(" LEDpin="); DBG.print(digitalRead(PIN_TX_LED));
  DBG.print(" PB12in="); DBG.print(digitalRead(PIN_PB12_UNUSED));
  DBG.print(" LEDreq="); DBG.print(g_txLedRequested ? 1 : 0);
  DBG.print(" DCLK/TXIN="); DBG.print(digitalRead(PIN_ADF_DCLK));
  DBG.print(" DATA/HiZ="); DBG.print(digitalRead(PIN_ADF_DATA));
  DBG.print(" MUX="); DBG.print(digitalRead(PIN_ADF_MUX));
  DBG.print(digitalRead(PIN_ADF_MUX) ? "(LOCK?)" : "(UNLOCK?)");
  DBG.print(" payloadBits="); DBG.print(g_payloadBitLen);
  DBG.print(" frameBits="); DBG.println(g_frameBitLen);
}

// -----------------------------------------------------------------------------
// Radio start/stop
// -----------------------------------------------------------------------------
static bool waitLock(uint16_t timeoutMs) {
  uint32_t start = millis();
  while ((uint16_t)(millis() - start) < timeoutMs) {
    if (digitalRead(PIN_ADF_MUX)) return true;
    delay(1);
  }
  return digitalRead(PIN_ADF_MUX) ? true : false;
}

static bool programRadioBase(bool paEnable, uint8_t r15TestMode) {
  // r15TestMode: 0=normal external data. 1..6 = ADF7021 internal test pattern.
  ceReset();
  dclkAsOutputLow();
  dataPinHiZ();

  adfWriteReg(buildR1_external_vco_div2()); delay(3);
  adfWriteReg(buildR15_normal()); delay(3);
  adfWriteReg(buildR3_9600()); delay(3);
  adfWriteReg(buildR2_tx(paEnable)); delay(3);
  adfWriteReg(buildR0(g_rfHz)); delay(3);
  if (r15TestMode) {
    adfWriteReg(buildR15_tx_test(r15TestMode));
    delay(3);
  }

  bool locked = waitLock(20);
  if (!locked) {
    DBG.println("WARN: MUX lock detect did not go high.");
  }
  return locked;
}

static void radioOff(bool reportStatus) {
  setPaActive(false);
  dclkAsOutputLow();
  digitalWrite(PIN_ADF_DCLK, LOW);
  dataPinHiZ();
  adfWriteReg(buildR15_normal());
  adfWriteReg(buildR2_tx(false));
  dclkAsInputPulldown();
  g_busy = false;
  g_mode = MODE_OFF;
  if (reportStatus) {
    DBG.println("TX OFF");
    status();
  }
}

// -----------------------------------------------------------------------------
// NMEA / AIS parsing and framing
// -----------------------------------------------------------------------------
static int ais6(char c) {
  int v = (int)c - 48;
  if (v > 40) v -= 8;
  if (v < 0 || v > 63) return -1;
  return v;
}

static bool nmeaChecksumOk(const char *s) {
  if (s[0] != '!') return false;
  const char *star = strchr(s, '*');
  if (!star || strlen(star) < 3) return false;

  uint8_t cs = 0;
  for (const char *p = s + 1; p < star; ++p) cs ^= (uint8_t)(*p);

  char hex[3] = { star[1], star[2], 0 };
  uint8_t got = (uint8_t)strtoul(hex, nullptr, 16);
  return cs == got;
}

static bool extractAisPayloadBits(const char *line, uint8_t *payloadBits, uint16_t &payloadBitLen) {
  payloadBitLen = 0;

  char tmp[MAX_NMEA_LINE];
  strncpy(tmp, line, sizeof(tmp));
  tmp[sizeof(tmp) - 1] = 0;

  char *star = strchr(tmp, '*');
  if (star) *star = 0;

  // Fields: !AIVDM,total,number,seq,channel,payload,fill
  char *fields[8] = {0};
  int count = 0;
  char *p = tmp;
  while (count < 8) {
    fields[count++] = p;
    char *comma = strchr(p, ',');
    if (!comma) break;
    *comma = 0;
    p = comma + 1;
  }

  if (count < 7) return false;
  if (strcmp(fields[0], "!AIVDM") != 0 && strcmp(fields[0], "!AIVDO") != 0) return false;

  int total = atoi(fields[1]);
  int number = atoi(fields[2]);
  if (total != 1 || number != 1) {
    DBG.println("Multipart AIVDM not supported in this first final sketch.");
    return false;
  }

  const char *payload = fields[5];
  int fillBits = atoi(fields[6]);
  if (fillBits < 0 || fillBits > 5) return false;

  for (const char *q = payload; *q; ++q) {
    int v = ais6(*q);
    if (v < 0) return false;
    for (int bit = 5; bit >= 0; --bit) {
      if (!appendBit(payloadBits, payloadBitLen, MAX_PAYLOAD_BITS, (v >> bit) & 1)) return false;
    }
  }

  if (payloadBitLen < (uint16_t)fillBits) return false;
  payloadBitLen -= fillBits;
  return true;
}

static uint16_t hdlcFcs16(const uint8_t *bits, uint16_t nbits) {
  // HDLC/X.25 style bit-oriented FCS: init 0xFFFF, polynomial reflected 0x8408, final complement.
  uint16_t fcs = 0xFFFF;
  for (uint16_t i = 0; i < nbits; i++) {
    uint8_t mix = (fcs ^ bits[i]) & 0x01;
    fcs >>= 1;
    if (mix) fcs ^= 0x8408;
  }
  return (uint16_t)~fcs;
}

static bool buildAisFrameBitsFromPayload(const uint8_t *payloadBits, uint16_t payloadLen,
                                         uint8_t *outBits, uint16_t &outLen) {
  outLen = 0;

  // AIVDM armoring exposes each AIS message octet MSB first, while HDLC sends
  // every octet least-significant bit first over the air. Without this
  // conversion a valid Message 18 starts as 0x48 on input but is received as
  // 0x12 (Message 4), and the MMSI is byte-wise bit-reversed as well.
  if ((payloadLen & 7U) != 0U) {
    DBG.println("AIS payload is not octet aligned");
    return false;
  }

  uint8_t dataFcs[MAX_FRAME_BITS];
  uint16_t dataFcsLen = 0;

  for (uint16_t byteOffset = 0; byteOffset < payloadLen; byteOffset += 8) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (!appendBit(dataFcs, dataFcsLen, MAX_FRAME_BITS,
                     payloadBits[byteOffset + 7U - bit])) return false;
    }
  }

  uint16_t fcs = hdlcFcs16(dataFcs, dataFcsLen);
  for (uint8_t i = 0; i < 16; i++) {
    if (!appendBit(dataFcs, dataFcsLen, MAX_FRAME_BITS, (fcs >> i) & 0x01)) return false;
  }

  // Bit-stuff payload + FCS only.
  uint8_t stuffed[MAX_FRAME_BITS];
  uint16_t stuffedLen = 0;
  uint8_t ones = 0;
  for (uint16_t i = 0; i < dataFcsLen; i++) {
    uint8_t b = dataFcs[i];
    if (!appendBit(stuffed, stuffedLen, MAX_FRAME_BITS, b)) return false;
    if (b) {
      ones++;
      if (ones == 5) {
        if (!appendBit(stuffed, stuffedLen, MAX_FRAME_BITS, 0)) return false;
        ones = 0;
      }
    } else {
      ones = 0;
    }
  }

  uint8_t raw[MAX_FRAME_BITS];
  uint16_t rawLen = 0;

  // Training sequence, then HDLC flag, stuffed data+FCS, end flag, and small guard.
  for (uint8_t i = 0; i < 24; i++) {
    if (!appendBit(raw, rawLen, MAX_FRAME_BITS, i & 1)) return false;
  }
  if (!appendFlag(raw, rawLen, MAX_FRAME_BITS)) return false;
  for (uint16_t i = 0; i < stuffedLen; i++) {
    if (!appendBit(raw, rawLen, MAX_FRAME_BITS, stuffed[i])) return false;
  }
  if (!appendFlag(raw, rawLen, MAX_FRAME_BITS)) return false;
  for (uint8_t i = 0; i < 16; i++) {
    if (!appendBit(raw, rawLen, MAX_FRAME_BITS, 0)) return false;
  }

  // NRZI encode for AIS/HDLC: 0 toggles, 1 keeps current level.
  uint8_t level = 0;
  for (uint16_t i = 0; i < rawLen; i++) {
    if (raw[i] == 0) level ^= 1;
    uint8_t out = level;
    if (g_invertTxData) out ^= 1;
    if (!appendBit(outBits, outLen, MAX_FRAME_BITS, out)) return false;
  }

  return true;
}

static bool buildFrameFromAivdm(const char *line, bool &checksumOk) {
  g_payloadBitLen = 0;
  g_frameBitLen = 0;

  checksumOk = nmeaChecksumOk(line);
  if (!checksumOk) {
    DBG.println("WARN: NMEA checksum error");
    if (g_strictChecksum) return false;
  }

  if (!extractAisPayloadBits(line, g_payloadBits, g_payloadBitLen)) {
    DBG.println("AIVDM parse error");
    return false;
  }

  if (!buildAisFrameBitsFromPayload(g_payloadBits, g_payloadBitLen, g_frameBits, g_frameBitLen)) {
    DBG.println("AIS frame build error");
    return false;
  }

  DBG.print("AIVDM OK: payloadBits="); DBG.print(g_payloadBitLen);
  DBG.print(" frameBits="); DBG.print(g_frameBitLen);
  DBG.print(" approxTxMs="); DBG.println((uint32_t)g_frameBitLen * 1000UL / 9600UL);
  return true;
}

// -----------------------------------------------------------------------------
// Transmit functions
// -----------------------------------------------------------------------------
static void sendTimedBitsBlocking(const uint8_t *bits, uint16_t nbits) {
  // Exact average bit period: 1,000,000 / 9600 = 104 + 1600/9600 us.
  uint32_t next = micros();
  uint16_t frac = 0;

  for (uint16_t i = 0; i < nbits; i++) {
    while ((int32_t)(micros() - next) < 0) {
      // busy wait for precise bit timing during short AIS burst
    }
    digitalWrite(PIN_ADF_DCLK, bits[i] ? HIGH : LOW);

    next += 104;
    frac += 1600;
    if (frac >= 9600) {
      next++;
      frac -= 9600;
    }
  }
}

static bool transmitBuiltAisFrame() {
  if (g_frameBitLen == 0) return false;
  if (g_busy) {
    DBG.println("TX busy");
    return false;
  }

  g_busy = true;
  g_mode = MODE_AIS_BURST;

  DBG.println("AIS TX start");

  bool locked = programRadioBase(true, 0);  // normal data mode
  if (!locked) {
    radioOff(false);
    DBG.println("AIS TX failed: PLL unlock");
    return false;
  }

  dclkAsOutputLow();
  setPaActive(true);

  // Very short PA/switch settle. Training bits follow immediately after.
  delayMicroseconds(300);
  sendTimedBitsBlocking(g_frameBits, g_frameBitLen);

  // GFSK transmit latency is several bit periods. Keep TX on briefly.
  delayMicroseconds(1200);

  radioOff(false);
  DBG.println("AIS TX done");
  return true;
}

static void startPn9() {
  if (g_busy) {
    DBG.println("TX busy");
    return;
  }
  DBG.println("PN9 start: ADF7021 internal R15 test mode 5. Use '0' to stop.");
  printN();
  g_busy = true;
  g_mode = MODE_PN9;
  programRadioBase(true, 5);
  dclkAsOutputLow();
  setPaActive(true);
  status();
}

static void startCarrier() {
  if (g_busy) {
    DBG.println("TX busy");
    return;
  }
  DBG.println("Carrier start: ADF7021 internal R15 test mode 1. Use '0' to stop.");
  printN();
  g_busy = true;
  g_mode = MODE_CARRIER;
  programRadioBase(true, 1);
  dclkAsOutputLow();
  setPaActive(true);
  status();
}

static void handleNmeaLine(const char *line) {
  if (strncmp(line, "!AIVDM", 6) != 0 && strncmp(line, "!AIVDO", 6) != 0) {
    DBG.print("USB ignored: "); DBG.println(line);
    USB_NMEA.println("TX NG: INVALID INPUT");
    return;
  }

  if (g_busy) {
    DBG.print("USB AIVDM ignored, TX busy: "); DBG.println(line);
    USB_NMEA.println("TX NG: BUSY");
    return;
  }

  DBG.print("USB RX: "); DBG.println(line);
  bool checksumOk = false;
  if (buildFrameFromAivdm(line, checksumOk)) {
    if (transmitBuiltAisFrame()) {
      oledShowSentence(line, "TX: done");
      USB_NMEA.println(checksumOk ? "TX OK" : "TX OK: CHECKSUM WARNING");
    } else {
      oledShowSentence(line, "TX: no lock");
      USB_NMEA.println("TX NG: NO LOCK");
    }
  } else {
    if (!checksumOk && g_strictChecksum) {
      oledShowSentence(line, "RX: checksum error");
      USB_NMEA.println("TX NG: CHECKSUM ERROR");
    } else {
      oledShowSentence(line, "RX: parse error");
      USB_NMEA.println("TX NG: PARSE ERROR");
    }
  }
}

// -----------------------------------------------------------------------------
// Services
// -----------------------------------------------------------------------------
static void serviceUsbNmea() {
  while (USB_NMEA.available()) {
    char c = (char)USB_NMEA.read();
    if (c == '\r' || c == '\n') {
      if (g_usbLineLen > 0) {
        g_usbLine[g_usbLineLen] = 0;
        handleNmeaLine(g_usbLine);
        g_usbLineLen = 0;
      }
    } else {
      if (g_usbLineLen < MAX_NMEA_LINE - 1) {
        g_usbLine[g_usbLineLen++] = c;
      } else {
        g_usbLineLen = 0;
        DBG.println("USB line buffer overflow");
        USB_NMEA.println("TX NG: LINE TOO LONG");
      }
    }
  }
}

static void scanReadback() {
  DBG.println("Readback cfg 0x10..0x1F");
  for (uint8_t cfg = 0x10; cfg <= 0x1F; cfg++) {
    uint16_t v = adfReadback(cfg);
    DBG.print("  cfg=0x"); DBG.print(cfg, HEX);
    DBG.print(" read=0x");
    if (v < 0x1000) DBG.print('0');
    if (v < 0x0100) DBG.print('0');
    if (v < 0x0010) DBG.print('0');
    DBG.println(v, HEX);
  }
}

static void help() {
  DBG.println("Debug commands on Serial1:");
  DBG.println("  ?  help");
  DBG.println("  s  status");
  DBG.println("  w  print N/register words");
  DBG.println("  e  readback cfg 0x10..0x1F");
  DBG.println("  A  set AIS CH A 161.975 MHz");
  DBG.println("  B  set AIS CH B 162.025 MHz");
  DBG.println("  p  start PN9 continuous test pattern, R15 mode 5");
  DBG.println("  c  start carrier continuous test, R15 mode 1");
  DBG.println("  l  PB13 LED ON test only, no RF change");
  DBG.println("  o  PB13 LED OFF test only, no RF change");
  DBG.println("  0  TX off / stop PN9 or carrier");
  DBG.println("  x  send built-in sample AIVDM once");
  DBG.println("  i  toggle TX data polarity inversion and resend sample if desired");
  DBG.println("  K  toggle strict NMEA checksum check, default warning-only");
  DBG.println("  v  cycle VCO_ADJUST 0..3");
  DBG.println("  j  cycle VCO_BIAS 1..4");
  DBG.println("  +  PA level up");
  DBG.println("  -  PA level down");
  DBG.println("USB CDC Serial accepts raw !AIVDM/!AIVDO lines only.");
}

static void handleDebugChar(char c) {
  if (c == '\r' || c == '\n' || c == ' ') return;

  switch (c) {
    case '?': help(); break;
    case 's': status(); break;
    case 'w': printN(); printWords(); break;
    case 'e': scanReadback(); break;
    case 'A': g_rfHz = RF_CH_A; DBG.println("RF set CH A 161.975 MHz"); printN(); status(); break;
    case 'B': g_rfHz = RF_CH_B; DBG.println("RF set CH B 162.025 MHz"); printN(); status(); break;
    case 'p': startPn9(); break;
    case 'c': startCarrier(); break;
    case 'l': DBG.println("PB13 LED forced ON test"); setTxLed(true); status(); break;
    case 'o': DBG.println("PB13 LED forced OFF test"); setTxLed(false); status(); break;
    case '0': radioOff(true); break;
    case 'x': {
      static const char sample[] = "!AIVDM,1,1,,A,15Muq@002>G?svP00<:O?vN60<0,0*7C";
      handleNmeaLine(sample);
      break;
    }
    case 'i':
      g_invertTxData = !g_invertTxData;
      DBG.print("TX data invert="); DBG.println(g_invertTxData ? 1 : 0);
      status();
      break;
    case 'K':
      g_strictChecksum = !g_strictChecksum;
      DBG.print("strict checksum="); DBG.println(g_strictChecksum ? 1 : 0);
      status();
      break;
    case 'v':
      g_vcoAdjust = (g_vcoAdjust + 1) & 0x03;
      DBG.print("VCO_ADJUST="); DBG.println(g_vcoAdjust);
      status();
      break;
    case 'j':
      g_vcoBias++;
      if (g_vcoBias > 4) g_vcoBias = 1;
      DBG.print("VCO_BIAS="); DBG.println(g_vcoBias);
      status();
      break;
    case '+':
      if (g_paLevel < 63) g_paLevel++;
      DBG.print("PAlevel="); DBG.println(g_paLevel);
      status();
      break;
    case '-':
      if (g_paLevel > 0) g_paLevel--;
      DBG.print("PAlevel="); DBG.println(g_paLevel);
      status();
      break;
    default:
      DBG.print("Unknown debug command: "); DBG.println(c);
      help();
      break;
  }
}

static void serviceDebugUart() {
  while (DBG.available()) {
    char c = (char)DBG.read();
    handleDebugChar(c);
  }
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------
void setup() {
  dataPinHiZ();
  pinMode(PIN_ADF_SDATA, OUTPUT);
  pinMode(PIN_ADF_CE, OUTPUT);
  pinMode(PIN_ADF_PAC, OUTPUT);
  pinMode(PIN_ADF_SLE, OUTPUT);
  pinMode(PIN_ADF_SREAD, INPUT);
  pinMode(PIN_ADF_SCLK, OUTPUT);
  pinMode(PIN_ADF_INTLK, INPUT);
  dclkAsInputPulldown();
  pinMode(PIN_ADF_MUX, INPUT);
  pinMode(PIN_PB12_UNUSED, INPUT);
  pinMode(PIN_TX_LED, OUTPUT);
  setTxLed(false);

  // Stabilize debug UART RX if adapter is removed.
  pinMode(PIN_UART_RX, INPUT_PULLUP);

  digitalWrite(PIN_ADF_CE, LOW);
  setPaActive(false);
  idleBus();

  USB_NMEA.begin(USB_BAUD);
  DBG.begin(DBG_BAUD);

  // Roger Clark Arduino_STM32 core:
  // I2C1 remapped to PB8=SCL / PB9=SDA by OLEDWire(1, I2C_REMAP).
  // The old core has no setSCL()/setSDA() methods.
  // Allow OLED power and module reset to settle on USB power-up.
  delay(800);
  OLEDWire.begin();
  g_oledOk = oledBegin();
  if (g_oledOk) {
    oledShowBoot();
  }
  pinMode(PIN_UART_RX, INPUT_PULLUP);  // re-apply after Serial1.begin()
  pinMode(PIN_PB12_UNUSED, INPUT);      // keep PB12 high-Z/unused

  DBG.println();
  DBG.println("Bluepill + RF7021SE/ADF7021 USB-AIVDM AIS TX final-ish test");
  DBG.println("USB CDC Serial: raw !AIVDM/!AIVDO input");
  DBG.println("Serial1 PA9/PA10: debug only, 115200 bps, PA10 pull-up enabled");
  DBG.println("RF default: 161.975 MHz, external VCO + RF_DIV2, R=2, VCOBIAS=2, VCOADJ=2");
  DBG.println("TX LED: PB13 active HIGH, linked to PAC/PA-TX ON; PB12 is input/unused");
  DBG.print("OLED: PB8=SCL PB9=SDA addr=0x3C status="); DBG.println(g_oledOk ? "OK" : "NOT FOUND");
  ledSelfTest();
  DBG.println("WARNING: dummy load / attenuator / shielded / direct-coupled setup only.");

  ceReset();
  radioOff(true);
  printN();
  printWords();
  help();
}

void loop() {
  serviceUsbNmea();
  serviceDebugUart();
}
