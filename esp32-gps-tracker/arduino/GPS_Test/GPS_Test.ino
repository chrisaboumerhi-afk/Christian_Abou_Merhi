/*
 * GPS-only test for the NEO-6M (no SIM800L needed).
 *
 *   pio run -e gps_test -t upload -t monitor
 *
 * Wiring: GPS TX -> GPIO16, GPS RX -> GPIO17, VCC -> 3V3, GND -> GND, 9600 baud.
 *
 * Prints a parsed summary every 2 s. Type 'r' + Enter in the Serial Monitor
 * to toggle printing of the raw NMEA sentences ($GPGGA, $GPRMC, ...).
 */
#include <Arduino.h>
#include <TinyGPSPlus.h>

static const int      GPS_RX_PIN = 16;
static const int      GPS_TX_PIN = 17;
static const uint32_t GPS_BAUD   = 9600;

HardwareSerial gpsSerial(2);
TinyGPSPlus    gps;
bool           showRaw = true;   // raw NMEA on by default: proves the wiring works

void printSummary() {
  Serial.println();
  Serial.printf("[GPS] chars=%lu sentences-with-fix=%lu bad-checksum=%lu\n",
                (unsigned long)gps.charsProcessed(),
                (unsigned long)gps.sentencesWithFix(),
                (unsigned long)gps.failedChecksum());

  if (gps.charsProcessed() < 10) {
    Serial.println("[GPS] No data! Check GPS TX -> GPIO16, 3V3/GND and baud rate.");
    return;
  }

  Serial.printf("[GPS] satellites=%u hdop=%.1f\n",
                gps.satellites.isValid() ? (unsigned)gps.satellites.value() : 0,
                gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);

  if (gps.location.isValid()) {
    Serial.printf("[GPS] FIX  lat=%.6f lon=%.6f alt=%.1f m speed=%.2f km/h (age %lu ms)\n",
                  gps.location.lat(), gps.location.lng(), gps.altitude.meters(),
                  gps.speed.kmph(), (unsigned long)gps.location.age());
    Serial.printf("[GPS] https://maps.google.com/?q=%.6f,%.6f\n",
                  gps.location.lat(), gps.location.lng());
  } else {
    Serial.println("[GPS] Waiting for GPS fix... (go outdoors / near a window, cold start can take 1-5 min)");
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    Serial.printf("[GPS] UTC %04u-%02u-%02u %02u:%02u:%02u\n",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("\n=== NEO-6M GPS test === (type 'r' to toggle raw NMEA)");
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
    if (showRaw) Serial.write(c);
  }

  while (Serial.available()) {
    if (tolower(Serial.read()) == 'r') {
      showRaw = !showRaw;
      Serial.printf("\n[GPS] raw NMEA %s\n", showRaw ? "ON" : "OFF");
    }
  }

  static uint32_t lastSummary = 0;
  if (millis() - lastSummary >= 2000) {
    lastSummary = millis();
    printSummary();
  }
}
