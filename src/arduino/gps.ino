#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// congig
static const uint32_t USB_BAUD = 115200;

static const int GPS_RX_PIN = 4; // Arduino RX 
static const int GPS_TX_PIN = 3; // Arduino TX  

// GPS module baud 
static const uint32_t GPS_BAUD = 9600;

// Publish rate: 1Hz is stable, 5Hz is buggy
static const int GPS_UPDATE_HZ = 1;


// SoftwareSerial to GPS
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
Adafruit_GPS GPS(&gpsSerial);

uint32_t lastFixMs = 0;

static void print_json_fix() {
  // GPS.latitudeDegrees / longitudeDegrees are float degrees when parsing is enabled.
  float lat = GPS.latitudeDegrees;
  float lon = GPS.longitudeDegrees;

  // Altitude in meters (from GGA)
  float alt_m = GPS.altitude;

  // Speed in knots; convert to m/s
  float speed_ms = GPS.speed * 0.514444f;

  // Course over ground in degrees
  float track_deg = GPS.angle;

  // Satellites
  int sats = GPS.satellites;

  // HDOP
  float hdop = 0.0f;
  #ifdef ADAFRUIT_GPS_LIBRARY_VERSION
  #endif
  hdop = GPS.HDOP;

  // Fix quality: 0 = invalid, 1 = GPS fix, 2 = DGPS
  int fix = (int)GPS.fix;
  int fixquality = (int)GPS.fixquality;

  // Timestamp (GPS time if available, else 0s)
  int hour = GPS.hour;
  int minute = GPS.minute;
  int seconds = GPS.seconds;
  int milliseconds = GPS.milliseconds;

  // JSON line
  Serial.print("{\"fix\":"); Serial.print(fix);
  Serial.print(",\"fixquality\":"); Serial.print(fixquality);
  Serial.print(",\"lat\":"); Serial.print(lat, 7);
  Serial.print(",\"lon\":"); Serial.print(lon, 7);
  Serial.print(",\"alt\":"); Serial.print(alt_m, 2);
  Serial.print(",\"speed_ms\":"); Serial.print(speed_ms, 3);
  Serial.print(",\"track_deg\":"); Serial.print(track_deg, 2);
  Serial.print(",\"sats\":"); Serial.print(sats);
  Serial.print(",\"hdop\":"); Serial.print(hdop, 2);
  Serial.print(",\"gps_time\":\"");
  if (GPS.fix) {
    // HH:MM:SS.mmm
    if (hour < 10) Serial.print("0"); Serial.print(hour); Serial.print(":");
    if (minute < 10) Serial.print("0"); Serial.print(minute); Serial.print(":");
    if (seconds < 10) Serial.print("0"); Serial.print(seconds); Serial.print(".");
    if (milliseconds < 100) Serial.print("0");
    if (milliseconds < 10) Serial.print("0");
    Serial.print(milliseconds);
  } else {
    Serial.print("00:00:00.000");
  }
  Serial.print("\"");
  Serial.println("}");
}

void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial) { delay(10); }

  gpsSerial.begin(GPS_BAUD);

  // Initialize GPS
  GPS.begin(GPS_BAUD);

  // RMC (speed/track/time) + GGA (fix/alt/sats)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // Update rate
  if (GPS_UPDATE_HZ == 1) GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  else if (GPS_UPDATE_HZ == 5) GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  else GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // future attenna support
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  Serial.println("{\"status\":\"gps_bridge_ready\"}");
}

void loop() {
  // Read characters from GPS continuously
  char c = GPS.read();

  // Parse new NMEA sentence when received
  if (GPS.newNMEAreceived()) {
    // debug raw NMEA:
    // Serial.println(GPS.lastNMEA());

    if (!GPS.parse(GPS.lastNMEA())) {
      return; // parse failed
    }

    // Only output when we have a fix, and at a reasonable rate
    // GPS.fix is true when fix is valid
    uint32_t now = millis();
    if (GPS.fix && (now - lastFixMs) > 200) { // ~5 Hz max output from bridge
      lastFixMs = now;
      print_json_fix();
    }
  }
}
