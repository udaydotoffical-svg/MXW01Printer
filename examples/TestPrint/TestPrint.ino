/*
  MXW01 test print -- Seeed XIAO ESP32S3

  Run this FIRST, before building anything else. It proves the protocol
  works on your printer. If this prints, the rest of the project is
  ordinary ESP32 work.

  Arduino IDE settings:
    Board:      XIAO_ESP32S3
    PSRAM:      OPI PSRAM        <-- required
    USB CDC On Boot: Enabled

  If BLE crashes at startup with a "block_locate_free" assert, roll the
  esp32 core back from 3.3.7 to 3.3.6; that regression was reported
  specifically on this board.
*/

#include <MXW01Printer.h>
#include <Fonts/FreeSansBold12pt7b.h>

// 200 rows is a short test strip (~2.5 cm). The real page uses 1200.
MXW01Printer printer(200);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\nMXW01 test print");

  if (!printer.begin()) {
    Serial.println("canvas alloc failed -- is PSRAM enabled?");
    return;
  }

  // Turn the printer on first, then reset the board.
  if (!printer.connect("MXW01")) {
    Serial.println("could not connect");
    return;
  }

  MXW01Status st = printer.getStatus();
  if (st.valid) {
    Serial.printf("battery %u%%, temp %uC, state %d, err %u\n",
                  st.battery, st.temperature, st.state, st.errorCode);
  } else {
    Serial.println("no status response -- continuing anyway");
  }

  printer.setIntensity(MXW01_DEFAULT_INTENSITY);
  delay(100);

  // --- Draw the test strip ---
  printer.clear();
  printer.setTextColor(1);

  printer.setFont(&FreeSansBold12pt7b);
  printer.setCursor(10, 40);
  printer.print("THE UDAY TIMES");

  printer.setFont(NULL);          // back to the built-in 5x7 font
  printer.setTextSize(1);
  printer.setCursor(10, 60);
  printer.print("MXW01 driver test");

  // A frame plus a bit of geometry, so bit order and orientation are obvious.
  printer.drawRect(4, 4, MXW01_WIDTH - 8, 192, 1);
  printer.fillRect(10, 80, 60, 20, 1);          // solid block, left side
  printer.drawLine(10, 110, MXW01_WIDTH - 12, 110, 1);
  printer.drawCircle(60, 150, 30, 1);
  printer.fillCircle(150, 150, 30, 1);

  // Corner marker: this square should come out at the TOP-LEFT of the
  // strip. If it lands bottom-right, the page printed upside down --
  // pass true as the second argument to printBuffer() instead.
  printer.fillRect(8, 8, 12, 12, 1);

  // rotate180 = false matched real MXW01 hardware: masthead first, upright.
  if (printer.printBuffer(200, false)) {
    Serial.println("done");
  } else {
    Serial.println("print failed");
  }

  printer.disconnect();
}

void loop() {}
