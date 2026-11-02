/*
 * ============================================================================
 *  ESP32 GPS Tracker with GSM (SIM800L) SMS reporting
 * ============================================================================
 *
 *  Hardware
 *    - ESP32 DevKit
 *    - NEO-6M GPS  on UART2 : GPS TX -> GPIO16 (RX2), GPS RX -> GPIO17 (TX2), 9600 baud, 3V3
 *    - SIM800L GSM on UART1 : SIM800L TX -> GPIO26, SIM800L RX -> GPIO27, RST -> GPIO25
 *      SIM800L powered from a separate 4 V / 2 A supply + 1000 uF capacitor, common GND.
 *
 *  !!! IMPORTANT – 2G ONLY !!!
 *    The SIM800L is a 2G (GSM/GPRS, 850/900/1800/1900 MHz) modem. It CANNOT use
 *    3G, 4G/LTE or 5G. Your local carrier MUST still operate a 2G/GSM network in
 *    your area, and the SIM card must be allowed on it. Many carriers have shut
 *    2G down – if the modem never registers ("searching" forever) this is the
 *    first thing to check.
 *
 *  What it does
 *    1. Parses GPS data (lat, lon, speed, altitude, UTC time, satellites) with TinyGPSPlus.
 *    2. Brings up the SIM800L with TinyGSM, printing SIM status, signal quality and
 *       network registration to the Serial Monitor (115200 baud).
 *    3. Every 30 s, if the GPS has a valid fix, sends an SMS containing a Google Maps link.
 *    4. Replies with the current position when an SMS containing "LOCATION" arrives.
 *    5. Without a GPS fix it prints "Waiting for GPS fix..." and sends nothing.
 *    6. Watches the modem; if it stops answering or loses the network it is hard
 *       reset through the RST pin and re-initialised.
 *
 *  The main loop never uses delay(): every periodic job is scheduled with millis()
 *  and the modem is driven by a small non-blocking state machine.
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
//  USER CONFIGURATION – edit these values
// ============================================================================

// Phone number that receives the periodic reports, in international format.
static const char OWNER_PHONE_NUMBER[] = "+10000000000";

// SIM card PIN. Leave empty ("") if the SIM has no PIN lock.
static const char SIM_PIN[] = "";

// Send a location SMS every SMS_REPORT_INTERVAL_MS while there is a GPS fix.
// NOTE: one SMS every 30 s is ~2880 SMS per day – check your SIM plan!
static const bool     PERIODIC_SMS_ENABLED   = true;
static const uint32_t SMS_REPORT_INTERVAL_MS = 30UL * 1000UL;

// Only answer "LOCATION" requests that come from OWNER_PHONE_NUMBER.
// Set to false to answer anyone who texts the tracker (privacy risk!).
static const bool ONLY_OWNER_CAN_REQUEST = true;

// The command word (case-insensitive) that triggers an immediate reply.
static const char LOCATION_COMMAND[] = "LOCATION";

// ============================================================================
//  Pin map and serial settings
// ============================================================================
static const int      GPS_RX_PIN  = 16;     // ESP32 RX2  <- GPS TX
static const int      GPS_TX_PIN  = 17;     // ESP32 TX2  -> GPS RX
static const uint32_t GPS_BAUD    = 9600;

static const int      GSM_RX_PIN  = 26;     // ESP32 RX   <- SIM800L TX
static const int      GSM_TX_PIN  = 27;     // ESP32 TX   -> SIM800L RX
static const int      GSM_RST_PIN = 25;     // ESP32 GPIO -> SIM800L RST (active LOW)
static const uint32_t GSM_BAUD    = 9600;   // SIM800L auto-bauds; 9600 is the safest

static const uint32_t SERIAL_BAUD = 115200; // USB Serial Monitor

// ============================================================================
//  Timing constants (all in milliseconds)
// ============================================================================
static const uint32_t GPS_STATUS_PRINT_MS   = 5000;   // print GPS status
static const uint32_t GPS_MAX_FIX_AGE_MS    = 5000;   // older location = no fix
static const uint32_t GPS_NO_DATA_WARN_MS   = 10000;  // warn if no NMEA after this

static const uint32_t GSM_RESET_PULSE_MS    = 200;    // RST low time (datasheet >= 105 ms)
static const uint32_t GSM_BOOT_TIME_MS      = 6000;   // wait after reset before talking
static const uint32_t GSM_AT_TIMEOUT_MS     = 20000;  // modem must answer AT within this
static const uint32_t GSM_SIM_TIMEOUT_MS    = 30000;  // SIM must be ready within this
static const uint32_t GSM_NET_TIMEOUT_MS    = 120000; // must register within this
static const uint32_t GSM_STEP_INTERVAL_MS  = 2000;   // poll period during init states
static const uint32_t GSM_HEALTH_CHECK_MS   = 15000;  // health check period when ready
static const uint32_t GSM_NET_LOST_MAX_MS   = 60000;  // tolerated network loss before reset
static const uint32_t SMS_INBOX_POLL_MS     = 5000;   // fallback inbox poll period

static const uint8_t  GSM_MAX_AT_FAILURES   = 3;      // failed health checks before reset
static const uint8_t  GSM_MAX_SMS_FAILURES  = 3;      // failed SMS sends before reset
static const uint8_t  MAX_SMS_PER_POLL      = 8;      // inbox messages handled per poll

// ============================================================================
//  Libraries
// ============================================================================
#define TINY_GSM_MODEM_SIM800        // must be defined before including TinyGSM
#define TINY_GSM_RX_BUFFER 1024

#include <TinyGPSPlus.h>
#include <TinyGsmClient.h>
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
#endif

// ============================================================================
//  Global objects
// ============================================================================
HardwareSerial gpsSerial(2);         // UART2 -> NEO-6M
HardwareSerial gsmSerial(1);         // UART1 -> SIM800L
TinyGPSPlus    gps;

#ifdef DUMP_AT_COMMANDS
StreamDebugger debugger(gsmSerial, Serial);
TinyGsm        modem(debugger);
Stream&        atStream = debugger;  // raw access to the modem (for SMS inbox / URCs)
#else
TinyGsm        modem(gsmSerial);
Stream&        atStream = gsmSerial;
#endif

// ----------------------------------------------------------------------------
//  GSM state machine
// ----------------------------------------------------------------------------
enum class GsmState : uint8_t {
  RESET_PULSE,    // RST held low
  BOOTING,        // RST released, waiting for the modem to boot
  WAIT_AT,        // waiting for "OK" to "AT"
  WAIT_SIM,       // waiting for the SIM to be ready
  WAIT_NETWORK,   // waiting for network registration
  READY           // registered, SMS can be sent/received
};

struct GsmContext {
  GsmState state            = GsmState::RESET_PULSE;
  uint32_t stateSince       = 0;     // millis() when the current state was entered
  uint32_t lastStep         = 0;     // millis() of the last poll inside a state
  uint32_t lastSmsPoll      = 0;
  uint32_t networkLostSince = 0;     // 0 = network OK
  uint8_t  atFailures       = 0;
  uint8_t  smsFailures      = 0;
  uint16_t resetCount       = 0;
  bool     smsCheckRequested = false;
};

GsmContext gsm;

struct IncomingSms {
  String sender;
  String body;
};

// ============================================================================
//  Small helpers
// ============================================================================

// Returns true (and restarts the timer) when `interval` ms elapsed since `last`.
// Using subtraction keeps it correct when millis() overflows after ~49 days.
bool intervalElapsed(uint32_t& last, uint32_t interval) {
  uint32_t now = millis();
  if (now - last >= interval) {
    last = now;
    return true;
  }
  return false;
}

const char* gsmStateName(GsmState s) {
  switch (s) {
    case GsmState::RESET_PULSE:  return "RESET_PULSE";
    case GsmState::BOOTING:      return "BOOTING";
    case GsmState::WAIT_AT:      return "WAIT_AT";
    case GsmState::WAIT_SIM:     return "WAIT_SIM";
    case GsmState::WAIT_NETWORK: return "WAIT_NETWORK";
    case GsmState::READY:        return "READY";
  }
  return "?";
}

const char* regStatusName(int reg) {
  switch (reg) {
    case 0:  return "not registered, not searching";
    case 1:  return "registered (home)";
    case 2:  return "searching...";
    case 3:  return "registration DENIED";
    case 4:  return "unknown";
    case 5:  return "registered (roaming)";
    default: return "no answer";
  }
}

bool isRegistered(int reg) { return reg == 1 || reg == 5; }

bool gsmReady() { return gsm.state == GsmState::READY; }

// Keeps only the digits of a phone number ("+961 70-123" -> "96170123").
String digitsOnly(const String& number) {
  String out;
  for (size_t i = 0; i < number.length(); i++) {
    if (isdigit((unsigned char)number[i])) out += number[i];
  }
  return out;
}

// Compares two phone numbers by their last 8 digits, so "+96170123456"
// and "70123456" are treated as the same number.
bool isSameNumber(const String& a, const String& b) {
  const size_t N = 8;
  String da = digitsOnly(a), db = digitsOnly(b);
  if (da.length() < N || db.length() < N) return da == db;
  return da.substring(da.length() - N) == db.substring(db.length() - N);
}

// Returns the n-th (0-based) "quoted" field of a modem response line.
String quotedField(const String& line, int n) {
  int start = -1;
  for (int i = 0; i <= n; i++) {
    start = line.indexOf('"', start + 1);
    if (start < 0) return "";
    int end = line.indexOf('"', start + 1);
    if (end < 0) return "";
    if (i == n) return line.substring(start + 1, end);
    start = end;
  }
  return "";
}

// ============================================================================
//  GPS
// ============================================================================

// Feeds every byte waiting on UART2 into the TinyGPSPlus parser.
// Called from loop() and also while we wait on the modem, so no NMEA is lost.
void feedGps() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

// A fix is usable only if the location is valid AND recently updated.
bool hasValidFix() {
  return gps.location.isValid() && gps.location.age() < GPS_MAX_FIX_AGE_MS;
}

// Formats the GPS UTC date/time as "YYYY-MM-DD HH:MM:SS".
String gpsUtcString() {
  if (!gps.date.isValid() || !gps.time.isValid()) return "n/a";
  char buf[32];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
           gps.date.year(), gps.date.month(), gps.date.day(),
           gps.time.hour(), gps.time.minute(), gps.time.second());
  return String(buf);
}

// Builds the SMS body: a Google Maps link plus a short summary (< 160 chars).
String buildLocationMessage(const char* title) {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "%s\nhttps://maps.google.com/?q=%.6f,%.6f\n"
           "Spd %.1f km/h Alt %.0f m Sats %u\nUTC %s",
           title,
           gps.location.lat(), gps.location.lng(),
           gps.speed.kmph(), gps.altitude.meters(),
           (unsigned)gps.satellites.value(),
           gpsUtcString().c_str());
  return String(buf);
}

// Prints the current GPS state to the Serial Monitor.
void printGpsStatus() {
  if (!hasValidFix()) {
    Serial.printf("Waiting for GPS fix... (satellites: %u, NMEA chars: %lu, bad checksums: %lu)\n",
                  gps.satellites.isValid() ? (unsigned)gps.satellites.value() : 0,
                  (unsigned long)gps.charsProcessed(),
                  (unsigned long)gps.failedChecksum());
    if (millis() > GPS_NO_DATA_WARN_MS && gps.charsProcessed() < 10) {
      Serial.println("[GPS] WARNING: no data from the GPS module. Check GPS TX -> GPIO16, "
                     "power (3V3/GND) and baud rate (9600).");
    }
    return;
  }

  Serial.println("------------------- GPS -------------------");
  Serial.printf("Latitude  : %.6f\n", gps.location.lat());
  Serial.printf("Longitude : %.6f\n", gps.location.lng());
  Serial.printf("Speed     : %.2f km/h\n", gps.speed.kmph());
  Serial.printf("Altitude  : %.1f m\n", gps.altitude.meters());
  Serial.printf("Satellites: %u\n", (unsigned)gps.satellites.value());
  Serial.printf("HDOP      : %.1f\n", gps.hdop.hdop());
  Serial.printf("UTC time  : %s\n", gpsUtcString().c_str());
  Serial.printf("Maps link : https://maps.google.com/?q=%.6f,%.6f\n",
                gps.location.lat(), gps.location.lng());
  Serial.println("-------------------------------------------");
}

// Periodic GPS status task (millis()-based).
void gpsStatusTask() {
  static uint32_t lastPrint = 0;
  if (intervalElapsed(lastPrint, GPS_STATUS_PRINT_MS)) {
    printGpsStatus();
  }
}

// ============================================================================
//  GSM – low level helpers
// ============================================================================

// Reads one line ("\r\n" terminated) from the modem. Keeps feeding the GPS
// parser while waiting. Returns false on timeout.
bool readModemLine(String& line, uint32_t timeoutMs) {
  line = "";
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (atStream.available()) {
      char c = atStream.read();
      if (c == '\r') continue;
      if (c == '\n') return true;
      if (line.length() < 200) line += c;
    }
    feedGps();
    yield();
  }
  return false;
}

// Discards anything the modem printed (used while it boots).
void drainModem() {
  while (atStream.available()) atStream.read();
}

void setGsmState(GsmState next) {
  Serial.printf("[GSM] State %s -> %s\n", gsmStateName(gsm.state), gsmStateName(next));
  gsm.state      = next;
  gsm.stateSince = millis();
  gsm.lastStep   = 0;
}

// Starts a hardware reset of the SIM800L (RST LOW). The state machine
// releases the pin and re-initialises the modem without blocking.
void gsmRequestReset(const char* reason) {
  gsm.resetCount++;
  Serial.printf("[GSM] *** Hardware reset #%u: %s ***\n", gsm.resetCount, reason);
  digitalWrite(GSM_RST_PIN, LOW);
  gsm.atFailures       = 0;
  gsm.smsFailures      = 0;
  gsm.networkLostSince = 0;
  setGsmState(GsmState::RESET_PULSE);
}

// Prints signal quality (CSQ 0..31, 99 = unknown) and the matching dBm value.
int printSignalQuality() {
  int csq = modem.getSignalQuality();
  if (csq == 99 || csq < 0) {
    Serial.println("[GSM] Signal quality: unknown (99) – no signal / antenna?");
  } else {
    const char* label = csq < 10 ? "poor" : csq < 15 ? "fair" : csq < 20 ? "good" : "excellent";
    Serial.printf("[GSM] Signal quality: %d/31 (%d dBm, %s)\n", csq, -113 + 2 * csq, label);
  }
  return csq;
}

// Prints the modem supply voltage. SIM800L needs 3.4–4.4 V and up to 2 A peaks.
void printSupplyVoltage() {
  int mv = modem.getBattVoltage();
  if (mv > 0) {
    Serial.printf("[GSM] Supply voltage: %d mV%s\n", mv,
                  mv < 3500 ? "  <-- LOW! Check the 4 V supply / wiring" : "");
  }
}

// Prints and returns the network registration status (AT+CREG?).
int printRegistration() {
  int reg = (int)modem.getRegistrationStatus();
  Serial.printf("[GSM] Network registration: %s\n", regStatusName(reg));
  if (reg == 3) {
    Serial.println("[GSM] Registration denied: SIM not allowed on 2G, or not activated.");
  }
  return reg;
}

// Configures SMS handling: text mode, GSM charset, and "+CMTI" notifications
// (new messages are stored on the SIM and announced with their index).
void configureSms() {
  modem.sendAT(GF("+CMGF=1"));            // text mode
  modem.waitResponse();
  modem.sendAT(GF("+CSCS=\"GSM\""));      // GSM 7-bit character set
  modem.waitResponse();
  modem.sendAT(GF("+CNMI=2,1,0,0,0"));    // store SMS and send +CMTI notification
  modem.waitResponse();
  Serial.println("[GSM] SMS configured (text mode, new-message notifications on)");
}

// ============================================================================
//  GSM – state machine steps
// ============================================================================

void gsmStepResetPulse() {
  if (millis() - gsm.stateSince >= GSM_RESET_PULSE_MS) {
    digitalWrite(GSM_RST_PIN, HIGH);       // release reset, modem starts booting
    setGsmState(GsmState::BOOTING);
  }
}

void gsmStepBooting() {
  drainModem();                            // ignore boot garbage / "RDY" etc.
  if (millis() - gsm.stateSince >= GSM_BOOT_TIME_MS) {
    setGsmState(GsmState::WAIT_AT);
  }
}

void gsmStepWaitAt() {
  if (!intervalElapsed(gsm.lastStep, 1000)) return;

  if (modem.testAT(500)) {
    Serial.println("[GSM] Modem answered AT. Initialising...");
    modem.init();                          // echo off, verbose errors, etc.
    Serial.printf("[GSM] Modem: %s\n", modem.getModemInfo().c_str());
    Serial.printf("[GSM] IMEI : %s\n", modem.getIMEI().c_str());
    printSupplyVoltage();
    setGsmState(GsmState::WAIT_SIM);
    return;
  }

  Serial.println("[GSM] No answer to AT yet...");
  if (millis() - gsm.stateSince > GSM_AT_TIMEOUT_MS) {
    gsmRequestReset("modem does not answer AT (check wiring TX/RX, power 4 V / 2 A)");
  }
}

void gsmStepWaitSim() {
  if (!intervalElapsed(gsm.lastStep, GSM_STEP_INTERVAL_MS)) return;

  SimStatus sim = modem.getSimStatus();
  switch (sim) {
    case SIM_READY:
      Serial.println("[GSM] SIM status: READY");
      Serial.printf("[GSM] SIM CCID: %s\n", modem.getSimCCID().c_str());
      configureSms();
      setGsmState(GsmState::WAIT_NETWORK);
      Serial.println("[GSM] Waiting for 2G network registration...");
      return;

    case SIM_LOCKED:
      Serial.println("[GSM] SIM status: LOCKED (PIN required)");
      if (strlen(SIM_PIN) > 0) {
        Serial.println("[GSM] Unlocking SIM with SIM_PIN...");
        modem.simUnlock(SIM_PIN);
      } else {
        Serial.println("[GSM] Set SIM_PIN at the top of main.cpp or remove the PIN lock.");
      }
      break;

    default:
      Serial.println("[GSM] SIM status: ERROR / not inserted");
      break;
  }

  if (millis() - gsm.stateSince > GSM_SIM_TIMEOUT_MS) {
    gsmRequestReset("SIM not ready");
  }
}

void gsmStepWaitNetwork() {
  if (!intervalElapsed(gsm.lastStep, GSM_STEP_INTERVAL_MS)) return;

  printSignalQuality();
  int reg = printRegistration();

  if (isRegistered(reg)) {
    Serial.printf("[GSM] Operator: %s\n", modem.getOperator().c_str());
    Serial.println("[GSM] ===== GSM READY =====");
    setGsmState(GsmState::READY);
    gsm.smsCheckRequested = true;          // handle SMS that arrived while offline
    return;
  }

  if (millis() - gsm.stateSince > GSM_NET_TIMEOUT_MS) {
    gsmRequestReset("no network registration (does your carrier still run 2G?)");
  }
}

// Handles one unsolicited line from the modem (URC).
void handleUrcLine(const String& line) {
  if (line.length() == 0 || line == "OK") return;

  if (line.startsWith("+CMTI:")) {
    Serial.println("[GSM] New SMS received");
    gsm.smsCheckRequested = true;
  } else if (line.indexOf("UNDER-VOLTAGE") >= 0 || line.indexOf("OVER-VOLTAGE") >= 0) {
    Serial.printf("[GSM] POWER WARNING: %s\n", line.c_str());
    if (line.indexOf("POWER DOWN") >= 0) gsmRequestReset("modem powered down (voltage)");
  } else if (line.indexOf("POWER DOWN") >= 0) {
    gsmRequestReset("modem reported POWER DOWN");
  } else if (line == "RDY" || line.startsWith("+CPIN: NOT")) {
    // The modem rebooted by itself (typically a brown-out during a TX burst).
    gsmRequestReset("modem restarted unexpectedly (power supply too weak?)");
  } else {
    Serial.printf("[GSM] URC: %s\n", line.c_str());
  }
}

// Reads unsolicited messages printed by the modem between commands.
void pollModemUrcs() {
  static String buffer;
  while (atStream.available()) {
    char c = atStream.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = buffer;
      buffer = "";
      handleUrcLine(line);
      if (!gsmReady()) return;             // a reset was requested
    } else if (buffer.length() < 128) {
      buffer += c;
    }
  }
}

// Periodic health check while READY: AT response, signal, registration.
void gsmHealthCheck() {
  if (!modem.testAT(1000)) {
    gsm.atFailures++;
    Serial.printf("[GSM] Health check: no answer to AT (%u/%u)\n",
                  gsm.atFailures, GSM_MAX_AT_FAILURES);
    if (gsm.atFailures >= GSM_MAX_AT_FAILURES) gsmRequestReset("modem stopped responding");
    return;
  }
  gsm.atFailures = 0;

  printSignalQuality();
  printSupplyVoltage();
  int reg = printRegistration();

  if (isRegistered(reg)) {
    gsm.networkLostSince = 0;
    return;
  }

  // Network lost: give the modem some time to re-register by itself.
  if (gsm.networkLostSince == 0) {
    gsm.networkLostSince = millis();
    Serial.println("[GSM] Network lost, waiting for automatic re-registration...");
  } else if (millis() - gsm.networkLostSince > GSM_NET_LOST_MAX_MS) {
    gsmRequestReset("network lost for too long");
  }
}

// ============================================================================
//  SMS
// ============================================================================

// Sends an SMS and tracks failures (repeated failures trigger a modem reset).
bool sendSms(const String& number, const String& text) {
  if (!gsmReady()) {
    Serial.println("[SMS] GSM not ready, SMS not sent");
    return false;
  }

  Serial.printf("[SMS] Sending to %s:\n%s\n", number.c_str(), text.c_str());
  bool ok = modem.sendSMS(number, text);

  if (ok) {
    gsm.smsFailures = 0;
    Serial.println("[SMS] Sent OK");
  } else {
    gsm.smsFailures++;
    Serial.printf("[SMS] FAILED (%u/%u)\n", gsm.smsFailures, GSM_MAX_SMS_FAILURES);
    if (gsm.smsFailures >= GSM_MAX_SMS_FAILURES) gsmRequestReset("SMS sending keeps failing");
  }
  return ok;
}

// Reacts to the text of one received SMS.
void handleSmsCommand(const String& sender, String body) {
  body.trim();
  Serial.printf("[SMS] From %s: \"%s\"\n", sender.c_str(), body.c_str());

  String command = body;
  command.toUpperCase();
  if (command != LOCATION_COMMAND) {
    Serial.println("[SMS] Not a known command, ignored");
    return;
  }

  if (ONLY_OWNER_CAN_REQUEST && !isSameNumber(sender, OWNER_PHONE_NUMBER)) {
    Serial.println("[SMS] LOCATION request from an unauthorised number, ignored");
    return;
  }

  if (!hasValidFix()) {
    Serial.println("Waiting for GPS fix... (LOCATION request received, no SMS sent)");
    return;
  }

  sendSms(sender, buildLocationMessage("Current location"));
}

// Lists unread SMS (AT+CMGL), handles them, then deletes read messages so the
// SIM storage never fills up. Messages are collected first and handled after
// the listing finished, because sending an SMS uses the same serial port.
void checkIncomingSms() {
  IncomingSms inbox[MAX_SMS_PER_POLL];
  uint8_t count = 0;
  bool expectBody = false;
  String line;

  modem.sendAT(GF("+CMGL=\"REC UNREAD\""));
  while (readModemLine(line, 5000)) {
    if (line == "OK" || line == "ERROR" || line.startsWith("+CMS ERROR")) break;

    if (line.startsWith("+CMGL:")) {
      // +CMGL: <idx>,"REC UNREAD","<sender>","","<date>"
      expectBody = (count < MAX_SMS_PER_POLL);
      if (expectBody) inbox[count].sender = quotedField(line, 1);
    } else if (line.startsWith("+CMTI:")) {
      gsm.smsCheckRequested = true;        // another SMS arrived meanwhile
    } else if (expectBody && line.length() > 0) {
      inbox[count++].body = line;          // first line of the SMS text
      expectBody = false;
    }
  }

  if (count == 0) return;

  modem.sendAT(GF("+CMGD=1,1"));           // delete all READ messages
  modem.waitResponse(5000);

  for (uint8_t i = 0; i < count && gsmReady(); i++) {
    handleSmsCommand(inbox[i].sender, inbox[i].body);
  }
}

// Inbox task: runs immediately after a +CMTI notification, and every few
// seconds as a fallback (TinyGSM may swallow a URC during another command).
void smsInboxTask() {
  if (gsm.smsCheckRequested || intervalElapsed(gsm.lastSmsPoll, SMS_INBOX_POLL_MS)) {
    gsm.smsCheckRequested = false;
    gsm.lastSmsPoll = millis();
    checkIncomingSms();
  }
}

void gsmStepReady() {
  pollModemUrcs();
  if (!gsmReady()) return;

  if (intervalElapsed(gsm.lastStep, GSM_HEALTH_CHECK_MS)) {
    gsmHealthCheck();
    if (!gsmReady()) return;
  }

  smsInboxTask();
}

// Runs one step of the GSM state machine. Never blocks for long.
void gsmUpdate() {
  switch (gsm.state) {
    case GsmState::RESET_PULSE:  gsmStepResetPulse();  break;
    case GsmState::BOOTING:      gsmStepBooting();     break;
    case GsmState::WAIT_AT:      gsmStepWaitAt();      break;
    case GsmState::WAIT_SIM:     gsmStepWaitSim();     break;
    case GsmState::WAIT_NETWORK: gsmStepWaitNetwork(); break;
    case GsmState::READY:        gsmStepReady();       break;
  }
}

// ============================================================================
//  Periodic location report
// ============================================================================
void periodicReportTask() {
  static uint32_t lastReport = 0;
  if (!intervalElapsed(lastReport, SMS_REPORT_INTERVAL_MS)) return;

  if (!hasValidFix()) {
    Serial.println("Waiting for GPS fix... (periodic SMS skipped)");
    return;
  }
  if (!PERIODIC_SMS_ENABLED) return;
  if (!gsmReady()) {
    Serial.println("[SMS] Periodic report skipped: GSM not ready");
    return;
  }
  sendSms(OWNER_PHONE_NUMBER, buildLocationMessage("Tracker update"));
}

// ============================================================================
//  Arduino entry points
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);                              // setup only: let USB serial settle
  Serial.println();
  Serial.println("==============================================");
  Serial.println(" ESP32 GPS + GSM Tracker");
  Serial.println(" NOTE: SIM800L is 2G only - carrier must support 2G");
  Serial.println("==============================================");

  // GPS on UART2 – large RX buffer so NMEA survives while the modem is busy.
  gpsSerial.setRxBufferSize(2048);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // SIM800L on UART1.
  gsmSerial.setRxBufferSize(1024);
  gsmSerial.begin(GSM_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

  // Start with a clean hardware reset of the modem.
  pinMode(GSM_RST_PIN, OUTPUT);
  digitalWrite(GSM_RST_PIN, HIGH);
  gsmRequestReset("power-on");
}

void loop() {
  feedGps();             // 1. parse incoming NMEA
  gsmUpdate();           // 2. drive the modem (init / recovery / SMS inbox)
  gpsStatusTask();       // 3. print GPS status every 5 s
  periodicReportTask();  // 4. send location SMS every 30 s (if fix)
}
