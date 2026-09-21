/*
  THE UDAY TIMES -- button-activated personal newspaper
  Seeed XIAO ESP32S3 + MXW01 thermal printer

  Press the button: the board wakes, pulls tech headlines and a tech quote
  over WiFi, renders a 384px-wide page, prints it over BLE, and goes back
  to sleep. Idle draw is microamps, so it runs a long time on a battery.

  ---------------------------------------------------------------- WIRING
    Button between D1 and GND. No resistor needed (internal pull-up).
    Printer powered on and NOT connected to the phone app.

  ------------------------------------------------------- ARDUINO SETTINGS
    Board:            XIAO_ESP32S3
    PSRAM:            OPI PSRAM          <-- required, page buffer is 57 KB
    USB CDC On Boot:  Enabled            (so Serial works over USB)

    If BLE asserts at startup (block_locate_free), roll the esp32 core
    back from 3.3.7 to 3.3.6; that regression hit this board specifically.

  ------------------------------------------------------------ WHAT'S REAL
    Confirmed on hardware:
      - FRAME status offsets (battery/temperature), not the documented ones
      - print mode 0x01 avoids the long blank trailer 0x00 ejects
      - rotate180 = false prints the masthead upright, first
      - 48-byte chunks at 15 ms is the known-good transfer rate
    Unverified: the news/quote APIs are reachable from your network, and
    Google Tasks (off by default; see USE_GOOGLE_TASKS below).
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <esp_sleep.h>
#include <MXW01Printer.h>

// ============================== CONFIG ==============================

const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";

#define BUTTON_PIN      D1        // button to GND; D0/D1/D3/D4 are safe
#define PRINTER_NAME    "MXW01"
#define MAX_ROWS        1200      // 15 cm at 8 dots/mm
#define INTENSITY       MXW01_DEFAULT_INTENSITY   // 0x8C; raise if faint
#define NEWS_COUNT      5
#define FLIP_180        false     // true if your print comes out upside down

// Owner name in the masthead footer line.
const char* OWNER = "Vadodara";

// India Standard Time, no DST.
const char* TZ_INFO   = "IST-5:30";
const char* NTP_POOL1 = "pool.ntp.org";
const char* NTP_POOL2 = "time.google.com";

// Google Tasks needs a one-time OAuth login on a laptop, then a refresh
// token pasted here. Leave this off until the rest works.
#define USE_GOOGLE_TASKS 0
const char* GOOGLE_CLIENT_ID     = "";
const char* GOOGLE_CLIENT_SECRET = "";
const char* GOOGLE_REFRESH_TOKEN = "";

// ============================ GLOBALS ===============================

MXW01Printer printer(MAX_ROWS);

String headlines[NEWS_COUNT];
int    headlineCount = 0;
String tasks[6];
int    taskCount = 0;
String quoteText, quoteBy;

// Survives deep sleep, so the footer can show the print number.
RTC_DATA_ATTR uint32_t printCount = 0;

// ======================== SMALL UTILITIES ===========================

// Thermal paper is ASCII-only in the built-in font. Convert the common
// smart punctuation APIs return, and drop anything else non-printable.
String toAscii(const String& in){
  String out;
  out.reserve(in.length());
  for (size_t i = 0; i < in.length(); i++){
    uint8_t c = in[i];
    if (c == 0xE2 && i + 2 < in.length()){       // UTF-8 general punctuation
      uint8_t b1 = in[i+1], b2 = in[i+2];
      if (b1 == 0x80){
        if (b2 == 0x98 || b2 == 0x99){ out += '\''; i += 2; continue; }
        if (b2 == 0x9C || b2 == 0x9D){ out += '"';  i += 2; continue; }
        if (b2 == 0x93 || b2 == 0x94){ out += '-';  i += 2; continue; }
        if (b2 == 0xA6){ out += "..."; i += 2; continue; }
      }
    }
    if (c == 0xC2 && i + 1 < in.length()){ i += 1; continue; }  // nbsp etc.
    if (c >= 0x20 && c < 0x7F) out += (char)c;
    else if (c == '\n' || c == '\t') out += ' ';
    else if (c >= 0x80) { /* drop */ }
  }
  out.trim();
  return out;
}

// Minimal JSON string-field reader. Enough for the few fields we want,
// without pulling in a JSON library for a handful of strings.
String jsonString(const String& src, const String& key, int fromIndex = 0){
  String pat = "\"" + key + "\"";
  int k = src.indexOf(pat, fromIndex);
  if (k < 0) return "";
  int c = src.indexOf(':', k + pat.length());
  if (c < 0) return "";
  int i = c + 1;
  while (i < (int)src.length() && isspace(src[i])) i++;
  if (i >= (int)src.length() || src[i] != '"') return "";
  i++;
  String out;
  while (i < (int)src.length()){
    char ch = src[i];
    if (ch == '\\' && i + 1 < (int)src.length()){
      char n = src[i+1];
      if      (n == 'n' || n == 't') out += ' ';
      else if (n == 'u' && i + 5 < (int)src.length()){
        long cp = strtol(src.substring(i+2, i+6).c_str(), nullptr, 16);
        if (cp == 0x2019 || cp == 0x2018) out += '\'';
        else if (cp == 0x201C || cp == 0x201D) out += '"';
        else if (cp == 0x2013 || cp == 0x2014) out += '-';
        else if (cp < 128) out += (char)cp;
        i += 6; continue;
      }
      else out += n;
      i += 2; continue;
    }
    if (ch == '"') break;
    out += ch;
    i++;
  }
  return out;
}

bool httpGet(const String& url, String& body, uint16_t timeoutMs = 8000){
  WiFiClientSecure client;
  client.setInsecure();            // no cert pinning; these are public feeds
  client.setTimeout(timeoutMs / 1000 + 1);
  HTTPClient http;
  http.setTimeout(timeoutMs);
  http.setConnectTimeout(timeoutMs);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK){
    Serial.printf("  HTTP %d for %s\n", code, url.substring(0, 48).c_str());
    http.end();
    return false;
  }
  body = http.getString();
  http.end();
  return true;
}

// ========================== CONTENT FETCH ===========================

// Tech headlines from the Hacker News API. JSON, no key, and light enough
// to parse on-device -- unlike RSS, which would need a proxy and an XML
// parser. Stories are filtered to real tech newsrooms.
void fetchHeadlines(){
  headlineCount = 0;
  const char* domains[] = { "techcrunch.com", "theverge.com",
                            "arstechnica.com", "engadget.com" };
  const int nDomains = sizeof(domains) / sizeof(domains[0]);

  time_t now = time(nullptr);
  long since = (long)now - 172800;          // last 48 hours

  for (int d = 0; d < nDomains && headlineCount < NEWS_COUNT; d++){
    String url = "https://hn.algolia.com/api/v1/search_by_date?tags=story&query=";
    url += domains[d];
    url += "&restrictSearchableAttributes=url&numericFilters=created_at_i%3E";
    url += String(since);
    url += "&hitsPerPage=3";

    String body;
    if (!httpGet(url, body)) continue;

    // Take up to two stories per domain so one site cannot fill the page.
    int idx = 0, taken = 0;
    while (taken < 2 && headlineCount < NEWS_COUNT){
      int t = body.indexOf("\"title\"", idx);
      if (t < 0) break;
      String title = toAscii(jsonString(body, "title", t));
      idx = t + 8;
      if (title.length() < 12 || title.length() > 110) continue;

      bool dup = false;
      for (int i = 0; i < headlineCount; i++) if (headlines[i] == title) dup = true;
      if (dup) continue;

      headlines[headlineCount++] = title;
      taken++;
    }
  }
  Serial.printf("  headlines: %d\n", headlineCount);
}

// Tech quote for the footer, same two sources the web page uses.
void fetchQuote(){
  quoteText = ""; quoteBy = "";
  String body;

  if (httpGet("https://api.quotable.io/quotes/random?tags=technology&maxLength=160", body)){
    String t = toAscii(jsonString(body, "content"));
    if (t.length() > 0 && t.length() <= 160){
      quoteText = t;
      quoteBy   = toAscii(jsonString(body, "author"));
      return;
    }
  }
  // Quote Garden sits on a free tier that can be slow to wake up.
  if (httpGet("https://quote-garden.onrender.com/api/v3/quotes/random?genre=technology",
              body, 12000)){
    String t = toAscii(jsonString(body, "quoteText"));
    if (t.length() > 0 && t.length() <= 160){
      quoteText = t;
      quoteBy   = toAscii(jsonString(body, "quoteAuthor"));
      if (quoteBy.equalsIgnoreCase("unknown")) quoteBy = "";
      return;
    }
  }
  // Both down: a built-in line, so the footer is never empty.
  quoteText = "Any sufficiently advanced technology is indistinguishable from magic.";
  quoteBy   = "Arthur C. Clarke";
}

#if USE_GOOGLE_TASKS
// Exchanges the stored refresh token for a short-lived access token.
// The Cloud app must be "In production"; test-user tokens expire after 7 days.
String googleAccessToken(){
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(10000);
  if (!http.begin(client, "https://oauth2.googleapis.com/token")) return "";
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String post = "client_id=" + String(GOOGLE_CLIENT_ID) +
                "&client_secret=" + String(GOOGLE_CLIENT_SECRET) +
                "&refresh_token=" + String(GOOGLE_REFRESH_TOKEN) +
                "&grant_type=refresh_token";
  int code = http.POST(post);
  String body = (code == HTTP_CODE_OK) ? http.getString() : "";
  http.end();
  if (code != HTTP_CODE_OK){ Serial.printf("  token HTTP %d\n", code); return ""; }
  return jsonString(body, "access_token");
}

void fetchTasks(){
  taskCount = 0;
  String token = googleAccessToken();
  if (!token.length()) return;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(10000);
  String url = "https://tasks.googleapis.com/tasks/v1/lists/@default/tasks"
               "?showCompleted=false&maxResults=6&fields=items(title)";
  if (!http.begin(client, url)) return;
  http.addHeader("Authorization", "Bearer " + token);
  int code = http.GET();
  String body = (code == HTTP_CODE_OK) ? http.getString() : "";
  http.end();
  if (code != HTTP_CODE_OK){ Serial.printf("  tasks HTTP %d\n", code); return; }

  int idx = 0;
  while (taskCount < 6){
    int t = body.indexOf("\"title\"", idx);
    if (t < 0) break;
    String title = toAscii(jsonString(body, "title", t));
    idx = t + 8;
    if (title.length()) tasks[taskCount++] = title;
  }
  Serial.printf("  tasks: %d\n", taskCount);
}
#else
void fetchTasks(){ taskCount = 0; }
#endif

// ============================ RENDERING =============================

int16_t rule(int16_t y, bool dashed){
  if (!dashed){ printer.fillRect(10, y, MXW01_WIDTH - 20, 2, 1); return y + 10; }
  for (int16_t x = 10; x < MXW01_WIDTH - 10; x += 12) printer.fillRect(x, y, 6, 1, 1);
  return y + 9;
}

// Draws the page into the canvas and returns its height in rows.
uint16_t renderPage(){
  printer.clear();
  printer.setTextColor(1);
  printer.setFont(NULL);

  int16_t y = 8;

  printer.setTextSize(2);
  y = printer.centerText("THE UDAY TIMES", y) + 4;

  printer.setTextSize(1);
  char line[64];
  struct tm t;
  if (getLocalTime(&t, 0)) strftime(line, sizeof(line), "%A, %d %B %Y", &t);
  else                     snprintf(line, sizeof(line), "%s", "Date unavailable");
  y = printer.centerText(line, y) + 2;

  snprintf(line, sizeof(line), "%s  -  No. %lu", OWNER, (unsigned long)printCount);
  y = printer.centerText(line, y) + 4;
  y = rule(y, false);

  if (taskCount){
    printer.setCursor(12, y);
    printer.print("TODAY'S TASKS");
    y += 14;
    for (int i = 0; i < taskCount; i++){
      printer.drawRect(13, y + 1, 9, 9, 1);
      y = printer.printWrapped(tasks[i].c_str(), 28, y, MXW01_WIDTH - 40) + 3;
    }
    y = rule(y + 2, true);
  }

  if (headlineCount){
    printer.setCursor(12, y);
    printer.print("TECH NEWS");
    y += 14;
    for (int i = 0; i < headlineCount; i++){
      printer.fillRect(14, y + 3, 4, 4, 1);
      y = printer.printWrapped(headlines[i].c_str(), 26, y, MXW01_WIDTH - 38) + 3;
    }
    y = rule(y + 2, true);
  } else {
    printer.setCursor(12, y);
    printer.print("No headlines today.");
    y += 16;
    y = rule(y, true);
  }

  if (quoteText.length()){
    y += 2;
    String q = "\"" + quoteText + "\"";
    y = printer.printWrapped(q.c_str(), 20, y, MXW01_WIDTH - 40) + 2;
    if (quoteBy.length()){
      String by = "- " + quoteBy;
      y = printer.centerText(by.c_str(), y) + 2;
    }
  }

  y += 10;
  if (y > MAX_ROWS) y = MAX_ROWS;     // hard 15 cm cap
  return (uint16_t)y;
}

// ========================= SLEEP AND BOOT ===========================

void goToSleep(){
  Serial.println("sleeping; press the button to print again");
  Serial.flush();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  // Wake when the button pulls the pin LOW.
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_PIN, 0);
  esp_deep_sleep_start();
}

bool connectWiFi(uint32_t timeoutMs = 20000){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) delay(250);
  if (WiFi.status() != WL_CONNECTED){ Serial.println("  WiFi failed"); return false; }
  Serial.printf("  WiFi ok, %s\n", WiFi.localIP().toString().c_str());

  configTzTime(TZ_INFO, NTP_POOL1, NTP_POOL2);
  struct tm t;
  getLocalTime(&t, 8000);            // needed for the date and the news filter
  return true;
}

void setup(){
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(400);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.printf("\nTHE UDAY TIMES -- %s\n",
                cause == ESP_SLEEP_WAKEUP_EXT0 ? "button press" : "power on");

  if (!printer.begin()){
    Serial.println("canvas alloc failed -- is PSRAM set to OPI?");
    goToSleep();
  }

  // 1) Content first, while the radio is already on.
  Serial.println("fetching...");
  bool online = connectWiFi();
  if (online){
    fetchTasks();
    fetchHeadlines();
    fetchQuote();
  } else {
    quoteText = "Any sufficiently advanced technology is indistinguishable from magic.";
    quoteBy   = "Arthur C. Clarke";
  }

  // 2) WiFi off BEFORE BLE. Running both at once is the classic way to
  //    run this board out of RAM mid-print.
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(250);

  // 3) Render, then print.
  printCount++;
  uint16_t rows = renderPage();
  Serial.printf("page: %u rows (%.1f cm)\n", rows, rows / 80.0);

  if (!printer.connect(PRINTER_NAME)){
    Serial.println("printer not found -- is it on, and not paired to the app?");
    goToSleep();
  }

  MXW01Status st = printer.getStatus();
  if (st.valid){
    Serial.printf("printer: %s\n", printer.statusText().c_str());
    if (st.battery > 0 && st.battery < 15)
      Serial.println("warning: printer battery low, print may be faint");
  }

  printer.setIntensity(INTENSITY);
  if (printer.printBuffer(rows, FLIP_180)) Serial.println("printed");
  else Serial.printf("print failed: %s\n", printer.statusText().c_str());

  printer.disconnect();
  goToSleep();
}

void loop(){}   // never runs; setup() ends in deep sleep
