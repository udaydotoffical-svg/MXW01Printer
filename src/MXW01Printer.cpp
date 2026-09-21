#include "MXW01Printer.h"

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ---- BLE UUIDs -------------------------------------------------------------
// Main service; some stacks report af30 instead of ae30, so try both.
static BLEUUID SERVICE_AE30("0000ae30-0000-1000-8000-00805f9b34fb");
static BLEUUID SERVICE_AF30("0000af30-0000-1000-8000-00805f9b34fb");
static BLEUUID CHAR_CONTROL("0000ae01-0000-1000-8000-00805f9b34fb");  // write
static BLEUUID CHAR_NOTIFY ("0000ae02-0000-1000-8000-00805f9b34fb");  // notify
static BLEUUID CHAR_DATA   ("0000ae03-0000-1000-8000-00805f9b34fb");  // write

static BLEClient*               g_client  = nullptr;
static BLERemoteCharacteristic* g_control = nullptr;
static BLERemoteCharacteristic* g_notify  = nullptr;
static BLERemoteCharacteristic* g_data    = nullptr;
static MXW01Printer*            g_instance = nullptr;
static BLEAdvertisedDevice*     g_found    = nullptr;

static void notifyTrampoline(BLERemoteCharacteristic* c, uint8_t* data,
                             size_t len, bool isNotify) {
  if (g_instance) g_instance->_onNotify(data, len);
}

class ScanCB : public BLEAdvertisedDeviceCallbacks {
 public:
  explicit ScanCB(const char* name) : _name(name) {}
  void onResult(BLEAdvertisedDevice dev) override {
    // getName() returns std::string on core 2.x and String on 3.x;
    // both expose c_str(), so compare through that for compatibility.
    if (dev.haveName() && strcmp(dev.getName().c_str(), _name) == 0) {
      if (g_found) delete g_found;
      g_found = new BLEAdvertisedDevice(dev);
      dev.getScan()->stop();
    }
  }
 private:
  const char* _name;
};

// ---- Construction ----------------------------------------------------------

MXW01Printer::MXW01Printer(uint16_t maxRows)
    : Adafruit_GFX(MXW01_WIDTH, maxRows), _maxRows(maxRows) {
  g_instance = this;
}

MXW01Printer::~MXW01Printer() {
  if (_buf) free(_buf);
  if (g_instance == this) g_instance = nullptr;
}

bool MXW01Printer::begin() {
  size_t bytes = (size_t)MXW01_BYTES_PER_ROW * _maxRows;

#if defined(BOARD_HAS_PSRAM) || defined(CONFIG_SPIRAM)
  if (psramFound()) _buf = (uint8_t*)ps_malloc(bytes);
#endif
  if (!_buf) _buf = (uint8_t*)malloc(bytes);

  if (!_buf) {
    if (debug) Serial.printf("[MXW01] failed to allocate %u bytes\n", (unsigned)bytes);
    return false;
  }
  memset(_buf, 0, bytes);
  if (debug) Serial.printf("[MXW01] canvas %ux%u, %u bytes\n",
                           MXW01_WIDTH, _maxRows, (unsigned)bytes);
  return true;
}

// ---- Protocol helpers ------------------------------------------------------

// CRC-8/DALLAS-MAXIM: poly 0x07, init 0x00, no reflection, no final xor.
// Computed over the PAYLOAD ONLY, not the header.
uint8_t MXW01Printer::crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

uint8_t MXW01Printer::reverseBits(uint8_t b) {
  b = (uint8_t)((b & 0xF0) >> 4 | (b & 0x0F) << 4);
  b = (uint8_t)((b & 0xCC) >> 2 | (b & 0x33) << 2);
  b = (uint8_t)((b & 0xAA) >> 1 | (b & 0x55) << 1);
  return b;
}

// Control packet: 22 21 <cmd> 00 <len_lo> <len_hi> <payload> <crc8> FF
bool MXW01Printer::writeControl(uint8_t cmd, const uint8_t* payload, size_t len) {
  if (!g_control) return false;

  uint8_t pkt[64];
  if (len + 8 > sizeof(pkt)) return false;

  size_t i = 0;
  pkt[i++] = 0x22;
  pkt[i++] = 0x21;
  pkt[i++] = cmd;
  pkt[i++] = 0x00;
  pkt[i++] = (uint8_t)(len & 0xFF);
  pkt[i++] = (uint8_t)(len >> 8);
  memcpy(&pkt[i], payload, len);
  i += len;
  pkt[i++] = crc8(payload, len);
  pkt[i++] = 0xFF;

  // catprinter.vercel.app uses controlChar.writeValue(), which is a
  // write WITH response; only the data characteristic uses
  // writeValueWithoutResponse(). Matching that exactly.
  g_control->writeValue(pkt, i, true);
  return true;
}

void MXW01Printer::_onNotify(uint8_t* data, size_t len) {
  if (len > sizeof(_notifyBuf)) len = sizeof(_notifyBuf);
  memcpy(_notifyBuf, data, len);
  _notifyLen = len;
  _notifyPending = true;
}

bool MXW01Printer::waitForNotify(uint8_t cmd, uint32_t timeoutMs,
                                 uint8_t* payloadOut, size_t* payloadLen) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (_notifyPending) {
      _notifyPending = false;

      // 22 21 <cmd> <unknown> <len_lo> <len_hi> <payload...> FF
      if (_notifyLen >= 7 && _notifyBuf[0] == 0x22 && _notifyBuf[1] == 0x21) {
        uint8_t rxCmd = _notifyBuf[2];
        size_t  plen  = (size_t)_notifyBuf[4] | ((size_t)_notifyBuf[5] << 8);
        if (rxCmd == cmd) {
          if (payloadOut && payloadLen) {
            if (plen > *payloadLen) plen = *payloadLen;
            if (6 + plen <= _notifyLen) memcpy(payloadOut, &_notifyBuf[6], plen);
            *payloadLen = plen;
          }
          return true;
        }
        if (debug) Serial.printf("[MXW01] ignoring notify 0x%02X (want 0x%02X)\n",
                                 rxCmd, cmd);
      }
    }
    delay(5);
  }
  if (debug) Serial.printf("[MXW01] timeout waiting for 0x%02X\n", cmd);
  return false;
}

// ---- Connection ------------------------------------------------------------

bool MXW01Printer::connect(const char* name, uint32_t scanSeconds) {
  if (debug) Serial.printf("[MXW01] scanning for \"%s\"...\n", name);

  BLEDevice::init("");
  BLEDevice::setMTU(247);  // room for larger chunks if you raise chunkSize

  if (g_found) { delete g_found; g_found = nullptr; }

  BLEScan* scan = BLEDevice::getScan();
  ScanCB cb(name);
  scan->setAdvertisedDeviceCallbacks(&cb);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  scan->start(scanSeconds, false);
  scan->stop();
  scan->clearResults();

  if (!g_found) {
    if (debug) Serial.println("[MXW01] printer not found (is it powered on?)");
    return false;
  }
  _address = g_found->getAddress().toString().c_str();

  g_client = BLEDevice::createClient();
  if (!g_client->connect(g_found)) {
    if (debug) Serial.println("[MXW01] connect failed");
    return false;
  }

  BLERemoteService* svc = g_client->getService(SERVICE_AE30);
  if (!svc) svc = g_client->getService(SERVICE_AF30);
  if (!svc) {
    if (debug) Serial.println("[MXW01] service ae30/af30 not found");
    g_client->disconnect();
    return false;
  }

  g_control = svc->getCharacteristic(CHAR_CONTROL);
  g_notify  = svc->getCharacteristic(CHAR_NOTIFY);
  g_data    = svc->getCharacteristic(CHAR_DATA);

  if (!g_control || !g_notify || !g_data) {
    if (debug) Serial.println("[MXW01] missing ae01/ae02/ae03 characteristics");
    g_client->disconnect();
    return false;
  }

  if (g_notify->canNotify()) g_notify->registerForNotify(notifyTrampoline);

  _connected = true;
  if (debug) Serial.printf("[MXW01] connected to %s\n", _address.c_str());
  return true;
}

void MXW01Printer::disconnect() {
  if (g_client && g_client->isConnected()) g_client->disconnect();
  _connected = false;
  g_control = g_notify = g_data = nullptr;
}

// ---- Drawing ---------------------------------------------------------------
// Pixels are stored directly in the printer's layout: 48 bytes per row,
// bit 0 of each byte is the LEFTMOST pixel of its group of 8. Note this is
// the opposite of Adafruit_GFX's usual MSB-first packing.

void MXW01Printer::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if (!_buf) return;

  // Bounds are checked against the ROTATED dimensions, then coordinates are
  // remapped using the RAW WIDTH/HEIGHT. This is exactly what GFXcanvas1
  // does; using _width/_height in the remap silently drops pixels at
  // rotation 1 and 3.
  if (x < 0 || y < 0 || x >= _width || y >= _height) return;

  int16_t t;
  switch (getRotation()) {
    case 1: t = x; x = WIDTH - 1 - y; y = t;               break;
    case 2: x = WIDTH - 1 - x; y = HEIGHT - 1 - y;         break;
    case 3: t = x; x = y; y = HEIGHT - 1 - t;              break;
  }

  uint32_t idx = (uint32_t)y * MXW01_BYTES_PER_ROW + (x >> 3);
  uint8_t  bit = 1 << (x & 7);  // bit 0 == leftmost

  if (color) _buf[idx] |= bit;
  else       _buf[idx] &= ~bit;
}

void MXW01Printer::fillScreen(uint16_t color) {
  if (!_buf) return;
  memset(_buf, color ? 0xFF : 0x00, (size_t)MXW01_BYTES_PER_ROW * _maxRows);
}

// ---- Grayscale + dithering -------------------------------------------------

void MXW01Printer::drawGrayscale(int16_t x0, int16_t y0, const uint8_t* gray,
                                 int16_t w, int16_t h, MXW01Dither mode) {
  if (!gray || w <= 0 || h <= 0) return;

  // 4x4 ordered dither matrix, scaled to 0-255.
  static const uint8_t bayer4[16] = {
      0, 136,  34, 170,
    204,  68, 238, 102,
     51, 187,  17, 153,
    255, 119, 221,  85
  };

  if (mode == MXW01_THRESHOLD || mode == MXW01_BAYER) {
    for (int16_t y = 0; y < h; y++) {
      for (int16_t x = 0; x < w; x++) {
        uint8_t v = gray[(int32_t)y * w + x];
        uint8_t t = (mode == MXW01_BAYER) ? bayer4[(y & 3) * 4 + (x & 3)] : 127;
        drawPixel(x0 + x, y0 + y, (v <= t) ? 1 : 0);
      }
    }
    return;
  }

  // Error-diffusion modes need a mutable copy of two or three rows.
  // We carry the error forward in int16 row buffers to keep RAM small.
  int16_t* cur  = (int16_t*)calloc(w, sizeof(int16_t));
  int16_t* nxt1 = (int16_t*)calloc(w, sizeof(int16_t));
  int16_t* nxt2 = (int16_t*)calloc(w, sizeof(int16_t));
  if (!cur || !nxt1 || !nxt2) { free(cur); free(nxt1); free(nxt2); return; }

  for (int16_t y = 0; y < h; y++) {
    for (int16_t x = 0; x < w; x++) cur[x] += gray[(int32_t)y * w + x];

    for (int16_t x = 0; x < w; x++) {
      int16_t old = cur[x];
      if (old < 0) old = 0;
      if (old > 255) old = 255;
      uint8_t bw = (old < 128) ? 0 : 255;
      drawPixel(x0 + x, y0 + y, bw ? 0 : 1);
      int16_t err = old - bw;

      if (mode == MXW01_FLOYD_STEINBERG) {
        if (x + 1 < w) cur[x + 1]  += err * 7 / 16;
        if (x > 0)     nxt1[x - 1] += err * 3 / 16;
        nxt1[x] += err * 5 / 16;
        if (x + 1 < w) nxt1[x + 1] += err * 1 / 16;
      } else {  // Atkinson: spread 6/8, deliberately losing some error
        int16_t e = err / 8;
        if (x + 1 < w) cur[x + 1]  += e;
        if (x + 2 < w) cur[x + 2]  += e;
        if (x > 0)     nxt1[x - 1] += e;
        nxt1[x] += e;
        if (x + 1 < w) nxt1[x + 1] += e;
        nxt2[x] += e;
      }
    }
    // Shift the row buffers up by one.
    int16_t* t = cur; cur = nxt1; nxt1 = nxt2; nxt2 = t;
    memset(nxt2, 0, w * sizeof(int16_t));
  }
  free(cur); free(nxt1); free(nxt2);
}

// ---- Text helpers ----------------------------------------------------------

int16_t MXW01Printer::printWrapped(const char* text, int16_t x, int16_t y,
                                   int16_t maxWidth, int16_t lineGap) {
  if (!text) return y;

  int16_t bx, by;
  uint16_t bw, bh;
  getTextBounds("Ag", 0, 0, &bx, &by, &bw, &bh);
  int16_t lineHeight = bh + lineGap;

  char line[96];
  size_t lineLen = 0;
  const char* p = text;

  while (*p) {
    // Grab the next word.
    const char* wordStart = p;
    while (*p && *p != ' ' && *p != '\n') p++;
    size_t wordLen = p - wordStart;
    // Clamp so a pathological word can never overrun line[]/candidate[].
    if (wordLen > sizeof(line) - 2) wordLen = sizeof(line) - 2;

    char candidate[96];
    size_t candLen = lineLen;
    memcpy(candidate, line, lineLen);
    if (candLen && candLen + 1 < sizeof(candidate)) candidate[candLen++] = ' ';
    if (candLen + wordLen < sizeof(candidate)) {
      memcpy(candidate + candLen, wordStart, wordLen);
      candLen += wordLen;
    }
    candidate[candLen] = '\0';

    getTextBounds(candidate, x, y, &bx, &by, &bw, &bh);

    if ((int16_t)bw > maxWidth && lineLen > 0) {
      line[lineLen] = '\0';
      setCursor(x, y);
      print(line);
      y += lineHeight;
      lineLen = 0;
      memcpy(line, wordStart, wordLen);
      lineLen = wordLen;
      line[lineLen] = '\0';
    } else {
      memcpy(line, candidate, candLen + 1);
      lineLen = candLen;
    }

    if (*p == '\n') {
      line[lineLen] = '\0';
      setCursor(x, y);
      print(line);
      y += lineHeight;
      lineLen = 0;
      line[0] = '\0';
    }
    if (*p) p++;  // skip the delimiter
  }

  if (lineLen) {
    line[lineLen] = '\0';
    setCursor(x, y);
    print(line);
    y += lineHeight;
  }
  return y;
}

int16_t MXW01Printer::centerText(const char* text, int16_t y) {
  int16_t bx, by;
  uint16_t bw, bh;
  getTextBounds(text, 0, y, &bx, &by, &bw, &bh);
  setCursor((MXW01_WIDTH - bw) / 2 - bx, y);
  print(text);
  return y + bh;
}

void MXW01Printer::hr(int16_t y, int16_t thickness, int16_t margin) {
  fillRect(margin, y, MXW01_WIDTH - margin * 2, thickness, 1);
}

void MXW01Printer::dashedHr(int16_t y, int16_t dash, int16_t margin) {
  for (int16_t x = margin; x < MXW01_WIDTH - margin; x += dash * 2) {
    fillRect(x, y, dash, 1, 1);
  }
}

// ---- Printing --------------------------------------------------------------

bool MXW01Printer::setIntensity(uint8_t intensity) {
  _intensity = intensity;
  uint8_t p[1] = { intensity };
  return writeControl(MXW01_CMD_INTENSITY, p, 1);
}

MXW01Status MXW01Printer::getStatus(uint32_t timeoutMs) {
  MXW01Status s;
  memset(&s, 0, sizeof(s));
  s.state = MXW01_UNKNOWN;
  if (!_connected) return s;

  clearNotify();
  uint8_t p[1] = { 0x00 };
  if (!writeControl(MXW01_CMD_STATUS, p, 1)) return s;

  uint8_t payload[sizeof(s.raw)];
  size_t  plen = sizeof(payload);
  if (!waitForNotify(MXW01_CMD_STATUS, timeoutMs, payload, &plen)) return s;

  s.rawLen = (uint8_t)plen;
  memcpy(s.raw, payload, plen);

  // Two published offset conventions, differing by the 6-byte header.
  // Read both; report the selected one and keep the other as *Alt.
  const uint8_t pStateI = 6,  pBattI = 9,  pTempI = 10, pFlagI = 12, pErrI = 13;
  const uint8_t fStateI = 0,  fBattI = 3,  fTempI = 4,  fFlagI = 6,  fErrI = 7;

  bool useFrame = (offsetBase == MXW01_OFFSETS_FRAME);
  uint8_t stateI = useFrame ? fStateI : pStateI;
  uint8_t battI  = useFrame ? fBattI  : pBattI;
  uint8_t tempI  = useFrame ? fTempI  : pTempI;
  uint8_t flagI  = useFrame ? fFlagI  : pFlagI;
  uint8_t errI   = useFrame ? fErrI   : pErrI;

  if (plen <= flagI) return s;   // too short to interpret at all
  s.valid = true;

  if (plen > battI) s.battery     = payload[battI];
  if (plen > tempI) s.temperature = payload[tempI];
  // The other convention's bytes, for comparison.
  uint8_t aBattI = useFrame ? pBattI : fBattI;
  uint8_t aTempI = useFrame ? pTempI : fTempI;
  if (plen > aBattI) s.batteryAlt     = payload[aBattI];
  if (plen > aTempI) s.temperatureAlt = payload[aTempI];

  if (payload[flagI] == 0) {
    s.errorCode = MXW01_ERR_NONE;
    switch (payload[stateI]) {
      case 0: s.state = MXW01_STANDBY;  break;
      case 1: s.state = MXW01_PRINTING; break;
      case 2: s.state = MXW01_FEEDING;  break;
      case 3: s.state = MXW01_EJECTING; break;
      default: s.state = MXW01_UNKNOWN; break;
    }
  } else {
    s.state = MXW01_ERROR;
    s.errorCode = (plen > errI) ? payload[errI] : 0xFF;
  }
  return s;
}

void MXW01Printer::dumpStatusPayload() {
  MXW01Status s = getStatus();
  if (!s.valid && s.rawLen == 0) { Serial.println("[MXW01] no status response"); return; }

  Serial.printf("[MXW01] A1 payload (%u bytes):\n  ", s.rawLen);
  for (uint8_t i = 0; i < s.rawLen; i++) Serial.printf("%02X ", s.raw[i]);
  Serial.println();
  Serial.println("  index: 0  1  2  3  4  5  6  7  8  9 10 11 12 13");
  Serial.printf("  PAYLOAD offsets -> state=%u batt=%u temp=%u flag=%u\n",
                s.rawLen > 6  ? s.raw[6]  : 0, s.rawLen > 9  ? s.raw[9]  : 0,
                s.rawLen > 10 ? s.raw[10] : 0, s.rawLen > 12 ? s.raw[12] : 0);
  Serial.printf("  FRAME   offsets -> state=%u batt=%u temp=%u flag=%u\n",
                s.rawLen > 0 ? s.raw[0] : 0, s.rawLen > 3 ? s.raw[3] : 0,
                s.rawLen > 4 ? s.raw[4] : 0, s.rawLen > 6 ? s.raw[6] : 0);
  Serial.println("  Whichever gives a sane battery %% and room-ish temp is correct;");
  Serial.println("  set printer.offsetBase accordingly.");
}

int16_t MXW01Printer::getBattery(uint32_t timeoutMs) {
  if (!_connected) return -1;

  // Dedicated battery command. Falls back to the status payload, which
  // also carries a battery byte, if this one does not answer.
  clearNotify();
  uint8_t p[1] = { 0x00 };
  if (writeControl(MXW01_CMD_BATTERY, p, 1)) {
    uint8_t payload[16];
    size_t  plen = sizeof(payload);
    if (waitForNotify(MXW01_CMD_BATTERY, timeoutMs, payload, &plen) && plen >= 1) {
      return payload[0];
    }
  }

  MXW01Status s = getStatus(timeoutMs);
  return s.valid ? (int16_t)s.battery : -1;
}

String MXW01Printer::getVersion(uint32_t timeoutMs) {
  if (!_connected) return String();

  clearNotify();
  uint8_t p[1] = { 0x00 };
  if (!writeControl(MXW01_CMD_VERSION, p, 1)) return String();

  uint8_t payload[32];
  size_t  plen = sizeof(payload);
  if (!waitForNotify(MXW01_CMD_VERSION, timeoutMs, payload, &plen)) return String();

  // Payload is a version string followed by unknown bytes; stop at the
  // first byte that is not printable ASCII.
  String v;
  for (size_t i = 0; i < plen; i++) {
    if (payload[i] < 0x20 || payload[i] > 0x7E) break;
    v += (char)payload[i];
  }
  return v;
}

int16_t MXW01Printer::getPrintType(uint32_t timeoutMs) {
  if (!_connected) return -1;

  clearNotify();
  uint8_t p[1] = { 0x00 };
  if (!writeControl(MXW01_CMD_PRINT_TYPE, p, 1)) return -1;

  uint8_t payload[16];
  size_t  plen = sizeof(payload);
  if (!waitForNotify(MXW01_CMD_PRINT_TYPE, timeoutMs, payload, &plen)) return -1;
  return (plen >= 1) ? (int16_t)payload[0] : -1;
}

bool MXW01Printer::cancel() {
  uint8_t p[1] = { 0x00 };
  if (!writeControl(MXW01_CMD_CANCEL, p, 1)) return false;
  delay(200);
  MXW01Status s = getStatus();
  return s.valid && s.state != MXW01_PRINTING;
}

bool MXW01Printer::isReady() {
  MXW01Status s = getStatus();
  return s.valid && s.state == MXW01_STANDBY && s.errorCode == MXW01_ERR_NONE;
}

String MXW01Printer::statusText() {
  MXW01Status s = getStatus();
  if (!s.valid) return "no response from printer";

  String out;
  switch (s.state) {
    case MXW01_STANDBY:  out = "standby";  break;
    case MXW01_PRINTING: out = "printing"; break;
    case MXW01_FEEDING:  out = "feeding paper";  break;
    case MXW01_EJECTING: out = "ejecting paper"; break;
    case MXW01_ERROR:
      switch (s.errorCode) {
        case MXW01_ERR_NO_PAPER:
        case MXW01_ERR_NO_PAPER_ALT: out = "error: out of paper"; break;
        case MXW01_ERR_OVERHEATED:   out = "error: overheated";   break;
        case MXW01_ERR_LOW_BATTERY:  out = "error: low battery";  break;
        default: out = "error code " + String(s.errorCode);       break;
      }
      break;
    default: out = "unknown state"; break;
  }
  out += ", battery " + String(s.battery) + "%";
  out += ", head " + String(s.temperature) + "C";
  return out;
}

bool MXW01Printer::sendRawCommand(uint8_t cmd, const uint8_t* payload, size_t len,
                                  uint8_t* responseOut, size_t* responseLen,
                                  uint32_t timeoutMs) {
  if (!_connected) return false;
  clearNotify();
  if (!writeControl(cmd, payload, len)) return false;
  return waitForNotify(cmd, timeoutMs, responseOut, responseLen);
}

// Sends one page. Split out so copies can reuse it.
bool MXW01Printer::sendPage(uint16_t rows, bool rotate180) {
  // Print request: <line_count LE16> 0x30 <mode>, mode 0x00 = 1bpp.
  clearNotify();
  uint8_t req[4] = { (uint8_t)(rows & 0xFF), (uint8_t)(rows >> 8), 0x30,
                     printMode };
  if (!writeControl(MXW01_CMD_PRINT, req, 4)) return false;

  uint8_t resp[8];
  size_t  rlen = sizeof(resp);
  if (!waitForNotify(MXW01_CMD_PRINT, 5000, resp, &rlen)) return false;
  if (rlen > 0 && resp[0] != 0x00) {
    if (debug) Serial.printf("[MXW01] print request rejected (0x%02X)\n", resp[0]);
    return false;
  }

  // Stream the image. Rotating 180 degrees means emitting rows bottom-up
  // with each row's pixel order reversed, which for LSB-first bytes means
  // walking the row's bytes backwards and bit-reversing each one.
  uint8_t  stage[256];
  uint16_t limit = chunkSize;
  if (limit > sizeof(stage)) limit = sizeof(stage);  // never overrun stage[]
  if (limit == 0) limit = 48;
  uint16_t staged = 0;
  uint32_t sent   = 0;

  for (uint16_t r = 0; r < rows; r++) {
    uint16_t srcRow = rotate180 ? (rows - 1 - r) : r;
    uint8_t* src = &_buf[(uint32_t)srcRow * MXW01_BYTES_PER_ROW];

    for (uint8_t b = 0; b < MXW01_BYTES_PER_ROW; b++) {
      stage[staged++] = rotate180 ? reverseBits(src[MXW01_BYTES_PER_ROW - 1 - b])
                                  : src[b];
      if (staged >= limit) {
        g_data->writeValue(stage, staged, false);
        sent += staged;
        staged = 0;
        delay(chunkDelay);
      }
    }
  }

  // Pad short pages up to the printer's apparent minimum.
  while (sent + staged < MXW01_MIN_DATA_BYTES) {
    stage[staged++] = 0x00;
    if (staged >= limit) {
      g_data->writeValue(stage, staged, false);
      sent += staged;
      staged = 0;
      delay(chunkDelay);
    }
  }
  if (staged) {
    g_data->writeValue(stage, staged, false);
    sent += staged;
    delay(chunkDelay);
  }
  if (debug) Serial.printf("[MXW01] sent %u bytes (%u rows)\n", (unsigned)sent, rows);

  // Flush, then wait for print-complete.
  clearNotify();
  uint8_t f[1] = { 0x00 };
  if (!writeControl(MXW01_CMD_FLUSH, f, 1)) return false;

  if (!waitForNotify(MXW01_CMD_COMPLETE, 20000)) {
    if (debug) Serial.println("[MXW01] no print-complete notification");
    return false;
  }
  return true;
}

bool MXW01Printer::printBuffer(uint16_t rows, bool rotate180, uint8_t copies) {
  if (!_connected || !_buf || !g_data) return false;
  if (rows == 0 || rows > _maxRows) rows = _maxRows;
  if (copies == 0) copies = 1;

  // Step 1: set intensity. The reference implementation re-sends this at
  // the start of every print job rather than relying on a prior call.
  setIntensity(_intensity);

  // Step 2: status. A reported error aborts; no response at all is only a
  // warning, and the job proceeds -- same as the reference.
  MXW01Status st = getStatus();
  if (st.valid && st.state == MXW01_ERROR) {
    if (debug) Serial.printf("[MXW01] not ready: %s\n", statusText().c_str());
    return false;
  }
  if (!st.valid && debug) Serial.println("[MXW01] no status response, proceeding anyway");

  for (uint8_t c = 0; c < copies; c++) {
    if (!sendPage(rows, rotate180)) return false;
    if (debug) Serial.printf("[MXW01] copy %u/%u done\n", c + 1, copies);
    if (c + 1 < copies) delay(300);
  }

  return true;
}

// A3 eject / A4 retract take a little-endian line count.
bool MXW01Printer::eject(uint16_t lines) {
  if (!_connected || lines == 0) return false;
  clearNotify();
  uint8_t pl[2] = { (uint8_t)(lines & 0xFF), (uint8_t)(lines >> 8) };
  if (!writeControl(MXW01_CMD_EJECT, pl, 2)) return false;
  waitForNotify(MXW01_CMD_EJECT, 2000);   // acknowledgement is advisory
  return true;
}

bool MXW01Printer::retract(uint16_t lines) {
  if (!_connected || lines == 0) return false;
  clearNotify();
  uint8_t pl[2] = { (uint8_t)(lines & 0xFF), (uint8_t)(lines >> 8) };
  if (!writeControl(MXW01_CMD_RETRACT, pl, 2)) return false;
  waitForNotify(MXW01_CMD_RETRACT, 2000);
  return true;
}

bool MXW01Printer::feed(uint16_t rows) {
  if (rows == 0) return true;
  if (useEjectCommand) return eject(rows);
  return feedBlankRows(rows);
}

// Fallback path: advances paper by printing blank rows. Slower and uses
// paper, but relies only on the core print sequence.
bool MXW01Printer::feedBlankRows(uint16_t rows) {
  if (!_connected || !g_data) return false;
  if (rows == 0) return true;

  clearNotify();
  uint8_t req[4] = { (uint8_t)(rows & 0xFF), (uint8_t)(rows >> 8), 0x30,
                     printMode };
  if (!writeControl(MXW01_CMD_PRINT, req, 4)) return false;

  uint8_t resp[8];
  size_t  rlen = sizeof(resp);
  if (!waitForNotify(MXW01_CMD_PRINT, 5000, resp, &rlen)) return false;
  if (rlen > 0 && resp[0] != 0x00) return false;

  uint8_t  zeros[192];
  memset(zeros, 0x00, sizeof(zeros));
  uint16_t limit = chunkSize;
  if (limit > sizeof(zeros)) limit = sizeof(zeros);
  if (limit == 0) limit = 48;

  uint32_t total = (uint32_t)rows * MXW01_BYTES_PER_ROW;
  if (total < MXW01_MIN_DATA_BYTES) total = MXW01_MIN_DATA_BYTES;
  for (uint32_t sent = 0; sent < total; ) {
    uint16_t n = (total - sent > limit) ? limit : (uint16_t)(total - sent);
    g_data->writeValue(zeros, n, false);
    sent += n;
    delay(chunkDelay);
  }

  clearNotify();
  uint8_t f[1] = { 0x00 };
  if (!writeControl(MXW01_CMD_FLUSH, f, 1)) return false;
  return waitForNotify(MXW01_CMD_COMPLETE, 20000);
}

bool MXW01Printer::printStatusReceipt() {
  MXW01Status s = getStatus();
  String fw = getVersion();

  clear();
  setFont(NULL);
  setTextColor(1);
  setTextSize(2);
  centerText("MXW01", 8);

  setTextSize(1);
  hr(32);

  int16_t y = 42;
  char line[48];

  snprintf(line, sizeof(line), "Battery:  %u%%", s.valid ? s.battery : 0);
  setCursor(12, y); print(line); y += 12;

  snprintf(line, sizeof(line), "Head:     %uC", s.valid ? s.temperature : 0);
  setCursor(12, y); print(line); y += 12;

  const char* st = "unknown";
  if (s.valid) {
    st = (s.state == MXW01_STANDBY)  ? "standby"
       : (s.state == MXW01_PRINTING) ? "printing"
       : (s.state == MXW01_FEEDING)  ? "feeding"
       : (s.state == MXW01_EJECTING) ? "ejecting"
       : (s.state == MXW01_ERROR)    ? "ERROR" : "unknown";
  }
  snprintf(line, sizeof(line), "State:    %s", st);
  setCursor(12, y); print(line); y += 12;

  if (s.valid && s.state == MXW01_ERROR) {
    snprintf(line, sizeof(line), "Err code: %u", s.errorCode);
    setCursor(12, y); print(line); y += 12;
  }

  snprintf(line, sizeof(line), "Firmware: %s", fw.length() ? fw.c_str() : "?");
  setCursor(12, y); print(line); y += 12;

  snprintf(line, sizeof(line), "Density:  0x%02X", _intensity);
  setCursor(12, y); print(line); y += 16;

  // Battery bar.
  uint8_t pct = s.valid ? s.battery : 0;
  if (pct > 100) pct = 100;
  drawRect(12, y, 200, 12, 1);
  fillRect(14, y + 2, (196 * pct) / 100, 8, 1);
  fillRect(212, y + 3, 4, 6, 1);   // battery nub
  y += 20;

  dashedHr(y);
  return printBuffer(y + 8, true);
}
