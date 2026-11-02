/*
 * GSM-only test for the SIM800L (no GPS needed).
 *
 *   pio run -e gsm_test -t upload -t monitor
 *
 * Wiring: SIM800L TX -> GPIO26, SIM800L RX -> GPIO27, RST -> GPIO25,
 *         VCC from the 4 V / 2 A supply (+1000 uF), GND shared with the ESP32.
 *
 * REMINDER: SIM800L is 2G only – your carrier must still run a 2G network.
 *
 * 1. Resets the modem with the RST pin.
 * 2. Runs a list of diagnostic AT commands and prints the answers.
 * 3. Becomes an AT terminal: every line you type is sent to the modem.
 *      ^Z            -> sends Ctrl+Z (ends the text of AT+CMGS)
 *      !sms <number> <text>  -> sends an SMS in one go
 *      !diag         -> repeats the diagnostic list
 *      !reset        -> hardware reset through RST
 */
#include <Arduino.h>

static const int      GSM_RX_PIN  = 26;
static const int      GSM_TX_PIN  = 27;
static const int      GSM_RST_PIN = 25;
static const uint32_t GSM_BAUD    = 9600;

HardwareSerial gsmSerial(1);
String inputLine;

// Forwards modem output to the Serial Monitor for `ms` milliseconds.
// (This is a test tool, so short blocking waits are acceptable here.)
void pumpModem(uint32_t ms) {
  uint32_t start = millis();
  while (millis() - start < ms) {
    while (gsmSerial.available()) Serial.write(gsmSerial.read());
    yield();
  }
}

void sendAt(const char* cmd, uint32_t waitMs = 1500) {
  Serial.printf("\n>> %s\n", cmd);
  gsmSerial.print(cmd);
  gsmSerial.print("\r");
  pumpModem(waitMs);
}

void resetModem() {
  Serial.println("\n[GSM] Hardware reset (RST low 200 ms), waiting 6 s for boot...");
  digitalWrite(GSM_RST_PIN, LOW);
  delay(200);
  digitalWrite(GSM_RST_PIN, HIGH);
  pumpModem(6000);   // you should see RDY / +CFUN: 1 / +CPIN: READY / Call Ready / SMS Ready
}

void runDiagnostics() {
  Serial.println("\n===== SIM800L diagnostics =====");
  sendAt("AT");             // must answer OK (also syncs the auto-baud)
  sendAt("ATE1");           // echo on, nicer for manual use
  sendAt("ATI");            // module name / firmware
  sendAt("AT+CBC");         // supply voltage: +CBC: 0,<pct>,<mV> -> want 3500..4400 mV
  sendAt("AT+CPIN?");       // +CPIN: READY = SIM OK
  sendAt("AT+CCID");        // SIM serial number
  sendAt("AT+CSQ");         // +CSQ: <rssi>,<ber>: rssi 10..31 good, 99 = no signal
  sendAt("AT+CREG?");       // +CREG: 0,1 (home) or 0,5 (roaming) = registered
  sendAt("AT+COPS?", 3000); // operator name
  Serial.println("\n===== end of diagnostics – type AT commands below =====");
  Serial.println("Shortcuts: ^Z | !sms <number> <text> | !diag | !reset");
}

void sendSms(const String& args) {
  int space = args.indexOf(' ');
  if (space < 0) {
    Serial.println("Usage: !sms +1234567890 hello");
    return;
  }
  String number = args.substring(0, space);
  String text   = args.substring(space + 1);

  sendAt("AT+CMGF=1");
  Serial.printf("\n>> AT+CMGS=\"%s\"\n", number.c_str());
  gsmSerial.printf("AT+CMGS=\"%s\"\r", number.c_str());
  pumpModem(1000);          // wait for the '>' prompt
  gsmSerial.print(text);
  gsmSerial.write(0x1A);    // Ctrl+Z ends the message
  pumpModem(10000);         // +CMGS: <ref> + OK = sent
}

void handleUserLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "^Z")              { gsmSerial.write(0x1A); return; }
  if (line == "!diag")           { runDiagnostics();      return; }
  if (line == "!reset")          { resetModem();          return; }
  if (line.startsWith("!sms "))  { sendSms(line.substring(5)); return; }

  gsmSerial.print(line);
  gsmSerial.print("\r");
}

void setup() {
  Serial.begin(115200);
  gsmSerial.setRxBufferSize(1024);
  gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  pinMode(GSM_RST_PIN, OUTPUT);
  digitalWrite(GSM_RST_PIN, HIGH);

  Serial.println("\n=== SIM800L GSM test === (2G network required)");
  resetModem();
  runDiagnostics();
}

void loop() {
  // Modem -> Serial Monitor
  while (gsmSerial.available()) Serial.write(gsmSerial.read());

  // Serial Monitor -> modem, line by line
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      handleUserLine(inputLine);
      inputLine = "";
    } else {
      inputLine += c;
    }
  }
}
