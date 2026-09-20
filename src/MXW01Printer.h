/*
  MXW01Printer v2.0 - Arduino/ESP32 BLE driver for the MXW01 thermal printer.

  The MXW01 does NOT speak the older 0x51 0x78 cat-printer protocol used by
  GB01/GT01/MX05/MX06 etc, so CatGFX and Cat-Printer will not drive it.
  This library implements the newer 0x22 0x21 framed protocol.

  Protocol reference:
    https://github.com/jeremy46231/MXW01-catprinter/blob/main/PROTOCOL.md
  Cross-checked against:
    https://docs.rs/crate/catprinter/0.1.0/source/src/protocol.rs
    https://github.com/MaikelChan/CatPrinterBLE

  Drawing is Adafruit_GFX compatible. The canvas is always 384 px wide and
  stores pixels directly in the printer's bit layout, so nothing is repacked
  at print time.

  Requires: ESP32 with BLE (ESP32, S3, C3 -- NOT S2), Adafruit_GFX.
  MIT licensed.
*/

#ifndef MXW01_PRINTER_H
#define MXW01_PRINTER_H

#include <Arduino.h>
#include <Adafruit_GFX.h>

// ---- Printer constants -----------------------------------------------------

#define MXW01_WIDTH         384  // print head width in dots (58mm @ ~203dpi)
#define MXW01_BYTES_PER_ROW  48  // 384 / 8
#define MXW01_DOTS_PER_MM     8  // so 15 cm == 1200 rows

// The printer appears to want at least 4320 bytes (90 rows). Short pages
// are zero padded on send.
#define MXW01_MIN_DATA_BYTES 4320

// 0x5D is the reference default but prints faint on most rolls. 0x8C is a
// better starting point; past ~0xB4 you risk scorching the paper.
#define MXW01_DEFAULT_INTENSITY 0x8C
#define MXW01_REFERENCE_INTENSITY 0x5D

// Command IDs (see README for which are confirmed vs unknown)
#define MXW01_CMD_STATUS     0xA1
#define MXW01_CMD_INTENSITY  0xA2
#define MXW01_CMD_EJECT      0xA3  // payload: line_count LE16
#define MXW01_CMD_RETRACT    0xA4  // payload: line_count LE16
#define MXW01_CMD_QUERY_COUNT 0xA7
#define MXW01_CMD_PRINT      0xA9
#define MXW01_CMD_COMPLETE   0xAA
#define MXW01_CMD_BATTERY    0xAB
#define MXW01_CMD_CANCEL     0xAC
#define MXW01_CMD_FLUSH      0xAD
#define MXW01_CMD_PRINT_TYPE 0xB0
#define MXW01_CMD_VERSION    0xB1

// Values 0-3 are the printer's own state codes.
enum MXW01State {
  MXW01_STANDBY  = 0,
  MXW01_PRINTING = 1,
  MXW01_FEEDING  = 2,
  MXW01_EJECTING = 3,
  MXW01_UNKNOWN  = 98,
  MXW01_ERROR    = 99
};

// Print modes, from the official app's options.
// 0x01 behaves like monochrome but ejects less paper afterwards.
#define MXW01_MODE_1BPP      0x00
#define MXW01_MODE_1BPP_ALT  0x01
#define MXW01_MODE_4BPP      0x02  // grayscale, 192 bytes/row, NOT implemented

enum MXW01Error {
  MXW01_ERR_NONE         = 0,
  MXW01_ERR_NO_PAPER     = 1,
  MXW01_ERR_OVERHEATED   = 4,
  MXW01_ERR_LOW_BATTERY  = 8,
  MXW01_ERR_NO_PAPER_ALT = 9
};

// Dithering modes for turning 8-bit grayscale into 1-bit.
enum MXW01Dither {
  MXW01_THRESHOLD,        // fastest, best for text and line art
  MXW01_FLOYD_STEINBERG,  // best general-purpose photo dithering
  MXW01_ATKINSON,         // higher contrast, classic Mac look
  MXW01_BAYER             // ordered, good for flat tones and textures
};

struct MXW01Status {
  bool       valid;        // false if no or malformed response
  MXW01State state;
  uint8_t    errorCode;    // 0 when OK
  uint8_t    battery;      // approximate percent
  uint8_t    temperature;  // approximate degrees C

  // Published implementations DISAGREE on where these live (see README).
  // Both candidate readings are exposed so you can see which is right on
  // your unit; `battery`/`temperature` above follow whichever `offsetBase`
  // selects.
  uint8_t    batteryAlt;      // the other candidate offset
  uint8_t    temperatureAlt;  // the other candidate offset
  uint8_t    raw[24];         // status payload as received
  uint8_t    rawLen;
};

// Which byte offsets to trust in the A1 status payload.
//   PAYLOAD: payload[6]/[9]/[10]/[12]/[13]
//            (dropalltables PROTOCOL.md, its own JS, the Rust crate)
//   FRAME:   the same numbers counted from the START OF THE FRAME, i.e.
//            payload[0]/[3]/[4]/[6]/[7] (MaikelChan's C#, and matches an
//            independent report of temperature at payload byte 4)
// These differ by the 6-byte header. Only your printer settles it.
enum MXW01OffsetBase { MXW01_OFFSETS_PAYLOAD, MXW01_OFFSETS_FRAME };

class MXW01Printer : public Adafruit_GFX {
 public:
  // maxRows = tallest page you will print. 1200 rows == 15 cm.
  // Costs 48 bytes per row (1200 rows == 57.6 KB, so use PSRAM).
  explicit MXW01Printer(uint16_t maxRows = 1200);
  ~MXW01Printer();

  bool begin();  // allocate the canvas; call once in setup()

  // --- Connection ---
  bool connect(const char* name = "MXW01", uint32_t scanSeconds = 10);
  void disconnect();
  bool isConnected() const { return _connected; }
  String address() const { return _address; }  // printer MAC, after connect()

  // --- Drawing (Adafruit_GFX). color: 1 = black, 0 = white ---
  void drawPixel(int16_t x, int16_t y, uint16_t color) override;
  void fillScreen(uint16_t color) override;
  void clear() { fillScreen(0); }

  // Draw 8-bit grayscale (0 = black, 255 = white) with dithering.
  // Handy for photos and anything rendered off-device.
  void drawGrayscale(int16_t x, int16_t y, const uint8_t* gray,
                     int16_t w, int16_t h,
                     MXW01Dither mode = MXW01_FLOYD_STEINBERG);

  // Word-wrapped text block. Returns the y coordinate just past the last
  // line so blocks can be stacked. Set a GFX font via setFont() first.
  int16_t printWrapped(const char* text, int16_t x, int16_t y,
                       int16_t maxWidth = MXW01_WIDTH - 20, int16_t lineGap = 2);

  // Common newspaper furniture.
  void    hr(int16_t y, int16_t thickness = 1, int16_t margin = 8);
  void    dashedHr(int16_t y, int16_t dash = 6, int16_t margin = 8);
  int16_t centerText(const char* text, int16_t y);

  // --- Printing ---
  // rows == 0 means "use maxRows". copies > 1 reprints the same page.
  // rotate180 defaults to FALSE: on real MXW01 hardware the masthead came
  // out first and upright with rotation off. The browser reference always
  // rotates, so flip this to true if your unit prints upside down.
  bool printBuffer(uint16_t rows = 0, bool rotate180 = false, uint8_t copies = 1);

  // Paper motion using the printer's own motor commands (A3/A4), taken
  // from MaikelChan's C# implementation.
  bool eject(uint16_t lines = 60);    // push paper out
  bool retract(uint16_t lines = 60);  // pull paper back in

  // feed() advances paper. By default it uses the A3 eject command; set
  // useEjectCommand=false to fall back to printing blank rows, which uses
  // only the core print path.
  bool feed(uint16_t rows = 60);
  bool feedMm(uint8_t mm) { return feed((uint16_t)mm * MXW01_DOTS_PER_MM); }
  bool useEjectCommand = true;
  bool feedBlankRows(uint16_t rows);

  // The printer ejects its own trailer after a job, so no extra feed by
  // default. Raise only if the tear bar cuts into your content.
  uint16_t autoFeedRows = 0;

  // Print mode sent in the A9 request.
  //   0x00 - the reference mode, but the printer then ejects a long
  //          blank trailer after every job.
  //   0x01 - same monochrome output, far less post-print eject.
  // Default 0x01 because the trailer wastes a lot of paper.
  uint8_t printMode = MXW01_MODE_1BPP_ALT;

  // --- Printer info and control ---
  // 0x00-0xFF, higher = darker. 0x5D is the reference default but prints
  // faint on many rolls; around 0x8C-0x96 is usually crisper. Past ~0xB4
  // you risk scorching the paper.
  bool        setIntensity(uint8_t intensity);
  uint8_t     intensity() const { return _intensity; }
  MXW01Status getStatus(uint32_t timeoutMs = 5000);
  int16_t     getBattery(uint32_t timeoutMs = 3000);    // percent, -1 on fail
  String      getVersion(uint32_t timeoutMs = 3000);    // firmware string
  int16_t     getPrintType(uint32_t timeoutMs = 3000);  // type byte, -1 on fail
  bool        cancel();                                 // abort current job
  // Which status-payload offsets to believe. See MXW01OffsetBase.
  // CONFIRMED ON HARDWARE: the FRAME convention gives the correct battery
  // reading on a real MXW01, so it is the default. The documented PAYLOAD
  // convention (PROTOCOL.md, the root JS, the Rust crate) is wrong by the
  // 6-byte header. Switch back if your firmware differs.
  MXW01OffsetBase offsetBase = MXW01_OFFSETS_FRAME;
  // Print the A1 payload as hex to Serial, to settle the offset question.
  void        dumpStatusPayload();
  String      statusText();                             // readable summary
  bool        isReady();                                // standby and no error

  // Print a small receipt with battery, temperature, state and firmware.
  bool printStatusReceipt();

  // Escape hatch for experimenting with undocumented opcodes.
  // Returns true if a notification with the same command ID came back.
  bool sendRawCommand(uint8_t cmd, const uint8_t* payload, size_t len,
                      uint8_t* responseOut = nullptr, size_t* responseLen = nullptr,
                      uint32_t timeoutMs = 2000);

  // --- Canvas access (printer bit order: bit 0 = leftmost pixel) ---
  uint8_t* buffer() { return _buf; }
  uint16_t maxRows() const { return _maxRows; }
  static uint16_t mmToRows(uint16_t mm) { return mm * MXW01_DOTS_PER_MM; }

  // --- Tuning ---
  // Defaults match catprinter.vercel.app (dropalltables/catprinter), the
  // browser implementation confirmed working on real MXW01 hardware:
  // one row (48 bytes) per write, 15 ms apart. A full 15 cm page then
  // takes roughly 18 s to transfer. Raising chunkSize to 180 is about 3x
  // faster and matches the Rust implementation, but relies on a
  // negotiated MTU of ~183+; try it only once printing already works.
  uint16_t chunkSize  = MXW01_BYTES_PER_ROW;  // 48
  uint8_t  chunkDelay = 15;    // ms between data writes; raise if bands drop
  bool     debug      = true;

  // Internal, public for the BLE notify callback.
  void _onNotify(uint8_t* data, size_t len);

 private:
  uint8_t*  _buf = nullptr;
  uint16_t  _maxRows;
  bool      _connected = false;
  uint8_t   _intensity = MXW01_DEFAULT_INTENSITY;
  String    _address;

  volatile bool _notifyPending = false;
  uint8_t   _notifyBuf[64];
  size_t    _notifyLen = 0;

  static uint8_t crc8(const uint8_t* data, size_t len);
  static uint8_t reverseBits(uint8_t b);

  bool writeControl(uint8_t cmd, const uint8_t* payload, size_t len);
  bool waitForNotify(uint8_t cmd, uint32_t timeoutMs,
                     uint8_t* payloadOut = nullptr, size_t* payloadLen = nullptr);
  void clearNotify() { _notifyPending = false; _notifyLen = 0; }
  bool sendPage(uint16_t rows, bool rotate180);
};

#endif  // MXW01_PRINTER_H
