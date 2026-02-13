#include <M5Cardputer.h>
#include <vector>
#include <algorithm>
#include <TinyGPSPlus.h>
#include <time.h>

#define APP_VERSION "2.0.1"


HardwareSerial GPS_Serial(2); // Use UART2 for GPS (Cap LoRa-1262 ATGM336H).
TinyGPSPlus gps;

struct SatData {
  String system;   // "GPS", "GLONASS", "Galileo", "BeiDou".
  int id;
  int elevation;   // 0-90°.
  int azimuth;     // 0-359°.
  int snr;         // 0-99.
  bool used;       // used in the fix.
  bool visible;    // visible in the last cycle.
};

// 3D globe projection result.
struct GlobeProj { int sx, sy; float z; };
std::vector<SatData> satellites;

struct GSVSequenceState {
    String system;
    int totalMsgs = 0;
    int lastMsgNum = 0;
    std::vector<int> currentVisible;
};
GSVSequenceState gsvStates[6]; // GPS, GLONASS, Galileo, BeiDou, QZSS, Mixed.
int gsvCount = 0;

// GGA fix quality and GSA fix mode data.
int ggaFixQuality = 0;    // 0=Invalid, 1=GPS, 2=DGPS, 4=RTK, 5=Float RTK.
int gsaFixMode = 1;       // 1=No fix, 2=2D, 3=3D.
float pdop = 99.9;
float vdop = 99.9;
float geoidHeight = 0.0;
bool geoidValid = false;
int qzssVisible = 0;

// ---- Screen system ----
enum ScreenID {
  SCR_MAIN = 0,
  SCR_SKY_VIEW,
  SCR_SIGNAL_BARS,
  SCR_FIX_SUMMARY,
  SCR_DASHBOARD,
  SCR_COORDINATES,
  SCR_BREADCRUMB,
  SCR_ALT_PROFILE,
  SCR_SPEED_GRAPH,
  SCR_TRIP_STATS,
  SCR_CONSTELLATION,
  SCR_NMEA_MONITOR,
  SCR_GPS_CLOCK,
  SCR_GPS_MAP,
  SCR_3D_GLOBE,
  SCR_COUNT
};

ScreenID currentScreen = SCR_MAIN;
bool autoSlideshow = false;
unsigned long slideshowInterval = 5000;
unsigned long lastSlideChange = 0;

const char* screenNames[] = {
  "Main", "Sky View", "Signal", "Fix Info",
  "Dashboard", "Coords", "Track",
  "Altitude", "Speed", "Trip", "Sats",
  "NMEA", "Clock", "Map", "Globe"
};

int mapZoom = 0;

// ---- IMU data ----
bool imuAvailable = false;
float imuAx = 0, imuAy = 0, imuAz = 0;   // Accelerometer (G)
float imuGx = 0, imuGy = 0, imuGz = 0;   // Gyroscope (deg/s)
float imuPitch = 0, imuRoll = 0;           // Tilt angles (degrees)
float imuGforce = 1.0;                     // Total G magnitude
float imuTemp = 0;                         // Sensor temperature (C)
float tripMaxG = 0;                        // Max G during trip

// ---- Track buffer ----
struct TrackPoint {
  float lat, lon;
  float altM;
  float speedKmph;
  uint32_t timestamp;
};
#define TRACK_MAX 120
TrackPoint trackBuf[TRACK_MAX];
int trackHead = 0;
int trackCount = 0;

// ---- Trip statistics ----
struct TripStats {
  float totalDistKm;
  float maxSpeedKmph;
  float maxAltM, minAltM;
  float totalAscentM, totalDescentM;
  float prevLat, prevLon, prevAlt;
  bool hasPrev;
  uint32_t startMillis;
  uint32_t movingMillis;
  uint32_t lastMovingCheck;
};
TripStats trip = {};

// ---- NMEA sentence buffer ----
#define NMEA_BUF_LINES 16
#define NMEA_BUF_WIDTH 84
char nmeaBuf[NMEA_BUF_LINES][NMEA_BUF_WIDTH];
int nmeaBufHead = 0;
int nmeaBufCount = 0;

// ---- Satellite counts (computed per frame) ----
struct SatCounts {
  int totalVisible, totalUsed, total;
  int gpsVis, glonassVis, galileoVis, beidouVis, qzssVis;
  int gpsUsed, glonassUsed, galileoUsed, beidouUsed, qzssUsed;
};

bool gpsSerial = false;
bool debugSerial = false;
bool nmeaSerial = false;
bool satListSerial = false;
bool hidePlotId = true;
bool hidePlotSystem = true;
bool openMenu = false;
bool helpMenu = false;
bool infoMenu = false;
bool configsMenu = false;
int configsMenuSel = 0;
String configsTmp[3] = {"", "", ""};  // 0 Rx, 1 Tx, 2 Baud.

int gpsRxPin = 15; // Cardputer ADV Rx pin <- GPS Tx pin (Cap LoRa-1262 EXT header).
int gpsTxPin = 13; // Cardputer ADV Tx pin <- GPS Rx pin (Cap LoRa-1262 EXT header).
int gpsBaud = 115200; // ATGM336H default baud rate.

enum GPSState { GPS_OFF, GPS_ON, GPS_ERR };
GPSState gpsSerialState = GPS_OFF;
const unsigned long GPS_TIMEOUT = 1000;
unsigned long lastValidGpsMillis = 0;

// Cardputer (1, 1.1, ADV) Display.
const int screenW = 240;
const int screenH = 135;

// Off-screen frame buffer for flicker-free rendering.
M5Canvas frameBuf(&M5Cardputer.Display);

// ==================================================================
//  Utility functions
// ==================================================================

/*    Haversine distance in km between two lat/lon points.
*/
float haversineKm(float lat1, float lon1, float lat2, float lon2) {
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371.0 * c;
}

/*    Count satellites by system.
*/
void countSatellites(SatCounts &c) {
  memset(&c, 0, sizeof(c));
  c.total = satellites.size();
  for (auto &sat : satellites) {
    if (sat.visible) {
      c.totalVisible++;
      if (sat.system == "GPS") c.gpsVis++;
      else if (sat.system == "GLONASS") c.glonassVis++;
      else if (sat.system == "Galileo") c.galileoVis++;
      else if (sat.system == "BeiDou") c.beidouVis++;
      else if (sat.system == "QZSS") c.qzssVis++;
    }
    if (sat.used) {
      c.totalUsed++;
      if (sat.system == "GPS") c.gpsUsed++;
      else if (sat.system == "GLONASS") c.glonassUsed++;
      else if (sat.system == "Galileo") c.galileoUsed++;
      else if (sat.system == "BeiDou") c.beidouUsed++;
      else if (sat.system == "QZSS") c.qzssUsed++;
    }
  }
}

/*    Cardinal direction from heading in degrees.
*/
const char* cardinalFromHeading(float heading) {
  const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int idx = (int)((heading + 22.5) / 45.0) % 8;
  return dirs[idx];
}

/*    Format milliseconds as HH:MM:SS.
*/
void formatDuration(uint32_t ms, char* buf, int bufSize) {
  uint32_t sec = ms / 1000;
  int h = sec / 3600;
  int m = (sec % 3600) / 60;
  int s = sec % 60;
  snprintf(buf, bufSize, "%02d:%02d:%02d", h, m, s);
}

/*    DOP quality label.
*/
const char* dopQuality(float dop) {
  if (dop < 2.0) return "Ideal";
  if (dop < 5.0) return "Good";
  if (dop < 10.0) return "Fair";
  return "Poor";
}

/*    Get system color for constellation display.
*/
uint16_t systemColor(const String &sys) {
  if (sys == "GPS") return TFT_GREEN;
  if (sys == "GLONASS") return TFT_RED;
  if (sys == "Galileo") return TFT_CYAN;
  if (sys == "BeiDou") return TFT_YELLOW;
  if (sys == "QZSS") return TFT_MAGENTA;
  return TFT_WHITE;
}

// ==================================================================
//  Track recording and trip stats
// ==================================================================

/*    Record a track point every 2 seconds when GPS has fix.
*/
void recordTrackPoint() {
  static uint32_t lastRecord = 0;
  if (!gps.location.isValid()) return;
  if (millis() - lastRecord < 2000) return;
  lastRecord = millis();

  TrackPoint &p = trackBuf[trackHead];
  p.lat = gps.location.lat();
  p.lon = gps.location.lng();
  p.altM = gps.altitude.isValid() ? gps.altitude.meters() : 0;
  p.speedKmph = gps.speed.kmph();
  p.timestamp = millis();

  trackHead = (trackHead + 1) % TRACK_MAX;
  if (trackCount < TRACK_MAX) trackCount++;
}

/*    Accumulate trip statistics.
*/
void updateTripStats() {
  if (!gps.location.isValid()) return;
  float lat = gps.location.lat();
  float lon = gps.location.lng();
  float alt = gps.altitude.isValid() ? gps.altitude.meters() : 0;
  float spd = gps.speed.kmph();

  if (!trip.hasPrev) {
    trip.prevLat = lat;
    trip.prevLon = lon;
    trip.prevAlt = alt;
    trip.hasPrev = true;
    trip.startMillis = millis();
    trip.lastMovingCheck = millis();
    trip.minAltM = alt;
    trip.maxAltM = alt;
    return;
  }

  float dist = haversineKm(trip.prevLat, trip.prevLon, lat, lon);
  if (dist > 0.001) // > 1 meter
    trip.totalDistKm += dist;

  if (spd > trip.maxSpeedKmph) trip.maxSpeedKmph = spd;
  if (alt > trip.maxAltM) trip.maxAltM = alt;
  if (alt < trip.minAltM) trip.minAltM = alt;

  float altDiff = alt - trip.prevAlt;
  if (altDiff > 0.5) trip.totalAscentM += altDiff;
  else if (altDiff < -0.5) trip.totalDescentM -= altDiff;

  uint32_t now = millis();
  if (spd > 1.0)
    trip.movingMillis += (now - trip.lastMovingCheck);
  trip.lastMovingCheck = now;

  trip.prevLat = lat;
  trip.prevLon = lon;
  trip.prevAlt = alt;
}

// ==================================================================
//  GPS serial functions
// ==================================================================

/*    Open or close the GPS UART serial console.
*/
void initGPSSerial(bool should_I) {
  if (should_I == true)
    GPS_Serial.begin(gpsBaud, SERIAL_8N1, gpsRxPin, gpsTxPin); // Start GPS UART.
  else
    GPS_Serial.end();
}

/*    Read the GPS seria and compose the NMEA sentence.
*/
void serialGPSRead() {
  static String nmeaLine = "";
  bool gotValidChar = false;
  GPSState prevState = gpsSerialState;
  if (GPS_Serial.available() == 0 && millis() - lastValidGpsMillis > GPS_TIMEOUT)
    gpsSerialState = GPS_ERR;
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    gps.encode(c);
    if (c != '\r' && c != '\n') gotValidChar = true;
    if (c == '\n') {
      nmeaDispatcher(nmeaLine);
      nmeaLine = "";
    } else if (c != '\r')
      nmeaLine += c;
  }
  if (gotValidChar) {
    lastValidGpsMillis = millis();
    gpsSerialState = GPS_ON;
  } else if (millis() - lastValidGpsMillis > GPS_TIMEOUT) {
    gpsSerialState = GPS_ERR;
  }
  if (gpsSerialState != prevState) {
    frameBuf.pushSprite(0, 0);
  }
}

// ==================================================================
//  NMEA parsing
// ==================================================================

/*    Read the NMEA sentence and dispatch to parsers.
*/
void nmeaDispatcher(const String &nmeaLine) {
  if (nmeaSerial)
    Serial.println(nmeaLine);
  // Trim line endings.
  String line = nmeaLine;
  line.trim();
  // Store in NMEA circular buffer for monitor screen.
  if (line.length() > 0 && line[0] == '$') {
    strncpy(nmeaBuf[nmeaBufHead], line.c_str(), NMEA_BUF_WIDTH - 1);
    nmeaBuf[nmeaBufHead][NMEA_BUF_WIDTH - 1] = '\0';
    nmeaBufHead = (nmeaBufHead + 1) % NMEA_BUF_LINES;
    if (nmeaBufCount < NMEA_BUF_LINES) nmeaBufCount++;
  }
  // Define NMEA handlers.
  struct NMEAHandler {
    const char* prefix;
    void (*parser)(const String&);
  };
  static NMEAHandler handlers[] = {
    {"$GPGSV", parseGSV},
    {"$GLGSV", parseGSV},
    {"$GAGSV", parseGSV},
    {"$BDGSV", parseGSV},
    {"$GQGSV", parseGSV},  // QZSS satellites.
    {"$GNGSV", parseGSV},
    {"$GPGSA", parseGSA},
    {"$GLGSA", parseGSA},
    {"$GAGSA", parseGSA},
    {"$BDGSA", parseGSA},
    {"$GQGSA", parseGSA},  // QZSS fix data.
    {"$GNGSA", parseGSA},
    {"$GPGGA", parseGGA},  // GPS fix data.
    {"$GNGGA", parseGGA}   // Multi-constellation fix data.
  };
  // Dispatch to the correct parser.
  for (auto &h : handlers) {
    if (line.startsWith(h.prefix)) {
      h.parser(line);
      break;
    }
  }
}

/*    Parse NMEA 0183 GSA sentence. (GNSS DOP and Active Satellites).
*     Mode (2D/3D), IDs of used satellites, PDOP/HDOP/VDOP.
*/
void parseGSA(const String &line) {
  int fieldNum = 0, lastIndex = 0;
  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line[i] == ',' || line[i] == '*') {
      String val = line.substring(lastIndex, i);
      lastIndex = i + 1;
      fieldNum++;
      // Field 2: Fix mode (1=No fix, 2=2D, 3=3D).
      if (fieldNum == 3 && val.length() > 0)
        gsaFixMode = val.toInt();
      // Fields 3-14: Satellite IDs used in fix.
      if (fieldNum >= 4 && fieldNum <= 15 && val.length() > 0) {
        int id = val.toInt();
        for (auto &sat : satellites) {
          if (sat.id == id) sat.used = true;
        }
      }
      // Field 15: PDOP.
      if (fieldNum == 16 && val.length() > 0)
        pdop = val.toFloat();
      // Field 16: HDOP (TinyGPSPlus handles this, skip).
      // Field 17: VDOP.
      if (fieldNum == 18 && val.length() > 0)
        vdop = val.toFloat();
    }
  }
}

/*    Parse NMEA 0183 GGA sentence. (Global Positioning System Fix Data).
*     Fix quality, geoid separation, number of satellites used.
*/
void parseGGA(const String &line) {
  int fieldNum = 0, lastIndex = 0;
  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line[i] == ',' || line[i] == '*') {
      String val = line.substring(lastIndex, i);
      lastIndex = i + 1;
      fieldNum++;
      // Field 6: Fix quality (0=Invalid, 1=GPS, 2=DGPS, 4=RTK, 5=Float RTK).
      if (fieldNum == 7 && val.length() > 0)
        ggaFixQuality = val.toInt();
      // Field 11: Geoid separation (meters).
      if (fieldNum == 12 && val.length() > 0) {
        geoidHeight = val.toFloat();
        geoidValid = true;
      }
    }
  }
}

/*    Parse NMEA 0183 GSV sentence. (GNSS Satellites in View).
*     Info on all visible satellites (ID, elevation, azimuth, SNR).
*/
void parseGSV(const String &line) {
  String system;
  if (line.startsWith("$GPGSV")) system = "GPS";
  else if (line.startsWith("$GLGSV")) system = "GLONASS";
  else if (line.startsWith("$GAGSV")) system = "Galileo";
  else if (line.startsWith("$BDGSV")) system = "BeiDou";
  else if (line.startsWith("$GQGSV")) system = "QZSS";
  else if (line.startsWith("$GNGSV")) system = "Mixed";
  else return;
  GSVSequenceState* state = getGSVState(system);
  if (!state) return;
  std::vector<String> fields;
  int lastIndex = 0;
  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line[i] == ',' || line[i] == '*') {
      fields.push_back(line.substring(lastIndex, i));
      lastIndex = i + 1;
    }
  }
  if (fields.size() < 4) return;
  int totalMsgs = fields[1].toInt();
  int msgNum    = fields[2].toInt();
  if (msgNum == 1 || state->totalMsgs != totalMsgs) {
    state->currentVisible.clear();
    state->totalMsgs = totalMsgs;
  }
  // Pars sats.
  for (size_t i = 4; i + 3 < fields.size(); i += 4) {
    SatData sat;
    sat.system = system;
    sat.id = fields[i].toInt();
    // Inverted BeiDou.
    if (system == "BeiDou") {
      sat.azimuth   = fields[i + 1].toInt();
      sat.elevation = fields[i + 2].toInt();
    } else {
      sat.elevation = fields[i + 1].toInt();
      sat.azimuth   = fields[i + 2].toInt();
    }
    sat.snr = fields[i + 3].toInt();
    sat.used = false;
    storeSatellite(sat);
    state->currentVisible.push_back(sat.id);
  }
  state->lastMsgNum = msgNum;
  if (msgNum == totalMsgs) {
    for (auto &s : satellites) {
      if (s.system == system) {
        s.visible = (std::find(state->currentVisible.begin(),state->currentVisible.end(),s.id) != state->currentVisible.end());
      }
    }
  }
}

/*    Stores GNSS satellite sequence states.
*/
GSVSequenceState* getGSVState(const String& system) {
  for (int i = 0; i < gsvCount; i++) {
    if (gsvStates[i].system == system)
      return &gsvStates[i];
  }
  if (gsvCount < 5) {
    gsvStates[gsvCount].system = system;
    return &gsvStates[gsvCount++];
  }
  return nullptr;
}

/*    Store satellite in a list.
*/
void storeSatellite(const SatData &sat) {
  for (auto &s : satellites) {
    if (s.system == sat.system && s.id == sat.id) {
      s.elevation = sat.elevation;
      s.azimuth   = sat.azimuth;
      s.snr       = sat.snr;
      return;
    }
  }
  satellites.push_back(sat);
}

// ==================================================================
//  Debug serial
// ==================================================================

/*    Open or close the USB serial console.
*/
void initDebugSerial(bool should_I) {
  if (should_I && !debugSerial) {
    debugSerial = true;
    Serial.println( "\n\n\n Initialited Cardputer ADV GPS INFO serial console!" );
  } else if (!should_I && debugSerial) {
    debugSerial = false;
  }
}

/*    Print in the USB Serial Console a List with all seen satellites.
*/
void serialConsoleSatsList() {
  if (!satListSerial) return;
  static uint32_t lastSerialSatsPrint = 0;
  if (millis() - lastSerialSatsPrint > 5000) {
    lastSerialSatsPrint = millis();
    if (satellites.empty()) return;
    // Sort by system and ID.
    std::sort(satellites.begin(), satellites.end(), [](const SatData &a, const SatData &b) {
        if (a.system != b.system) return a.system < b.system;
        return a.id < b.id;
    });
    Serial.println("------------------------------------");
    Serial.println("System    ID  Ele  Azi  SNR  Usd Vis");
    Serial.println("------------------------------------");
    for (auto &sat : satellites) {
      Serial.printf("%-8s %3d %4d %4d %4d   %c   %c\n", sat.system.c_str(), sat.id, sat.elevation, sat.azimuth, sat.snr, sat.used ? 'Y' : 'N', sat.visible ? 'V' : 'X');
      //                                                                                                                                         V = visibile, X = lost
    }
    Serial.println("------------------------------------");
  }
}

// ==================================================================
//  Screen drawing functions
// ==================================================================

/*    Draw the satellites sky plot graph (parametric).
*/
void drawSkyPlotAt(int x, int y, int w, int h, bool showIds, bool showSys) {
  int side = (w < h) ? w : h;
  int r = side / 2;
  int cx = x + w / 2;
  int cy = y + h / 2;
  frameBuf.drawRect(x-1, y-1, w+2, h+2, TFT_DARKGREY);
  frameBuf.fillRect(x, y, w, h, TFT_BLACK);
  // Ref circles.
  frameBuf.drawCircle(cx, cy, r, TFT_WHITE);
  frameBuf.drawCircle(cx, cy, r * 0.66, TFT_DARKGREY);
  frameBuf.drawCircle(cx, cy, r * 0.33, TFT_DARKGREY);
  frameBuf.drawLine(cx - r, cy, cx + r, cy, TFT_DARKGREY);
  frameBuf.drawLine(cx, cy - r, cx, cy + r, TFT_DARKGREY);
  // Cardinals label.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_LIGHTGREY);
  frameBuf.setCursor(cx - 3, cy - r + 4);  frameBuf.print("N");
  frameBuf.setCursor(cx - 3, cy + r - 10); frameBuf.print("S");
  frameBuf.setCursor(cx + r - 10, cy - 3); frameBuf.print("E");
  frameBuf.setCursor(cx - r + 4, cy - 3);  frameBuf.print("W");
  // Satellites.
  int dotR = (r > 50) ? 3 : 2;
  for (auto &sat : satellites) {
    float elev = constrain(sat.elevation, 0.0, 90.0);
    float az   = fmod(sat.azimuth + 360.0, 360.0);
    float rad = (90.0 - elev) / 90.0 * r;
    float radAz = radians(az);
    float sx = cx + rad * sin(radAz);
    float sy = cy - rad * cos(radAz);
    uint16_t color = TFT_RED;
    if (sat.used)
      color = TFT_GREEN;
    else if
      (sat.visible) color = TFT_YELLOW;
    frameBuf.fillCircle(sx, sy, dotR, color); // Satellite dot.
    if (showIds) // Satellite id.
    {
      frameBuf.setTextSize(0);
      frameBuf.setTextColor(TFT_BLACK);
      frameBuf.setCursor(sx + 4, sy - 4);
      frameBuf.printf("%d", sat.id);
      frameBuf.setTextColor(color);
      frameBuf.setCursor(sx + 5, sy - 3);
      frameBuf.printf("%d", sat.id);
    }
    if (showSys) // Satellite system.
    {
      const char* sys;
      if (sat.system == "GPS") sys = "Gp";
      else if (sat.system == "GLONASS") sys = "Gl";
      else if (sat.system == "Galileo") sys = "Ga";
      else if (sat.system == "BeiDou") sys = "Bd";
      else if (sat.system == "QZSS") sys = "Qz";
      else sys = "?";
      frameBuf.setTextSize(0);
      frameBuf.setTextColor(TFT_BLACK);
      frameBuf.setCursor(sx + 4, sy - 4);
      frameBuf.printf("%s", sys);
      frameBuf.setTextColor(color);
      frameBuf.setCursor(sx + 5, sy - 3);
      frameBuf.printf("%s", sys);
    }
  }
}

/*    Draw the satellites sky plot graph (original position for main screen).
*/
void drawSkyPlot() {
  drawSkyPlotAt(155, 2, 85, 131, !hidePlotId, !hidePlotSystem);
}

/*    Draw the satellites main data table.
*/
void drawSatelliteDataTab(){
  int x = 1;
  int y = 2;
  // Column 1.
  // Labels.
  const char* c1labels[] = { "Lat", "Lng", "Alt", "Spd", "Crs", "Date", "Time", "Fix" };
  // Values.
  char c1values[8][20];
  if (gps.location.isValid()) {sprintf(c1values[0], "%.6f", gps.location.lat());} else {sprintf(c1values[0], "NoFix");}
  if (gps.location.isValid()) {sprintf(c1values[1], "%.6f", gps.location.lng());} else {sprintf(c1values[1], "NoFix");}
  if (gps.altitude.isValid()) {sprintf(c1values[2], "%.1fm", gps.altitude.meters());} else {sprintf(c1values[2], "NoData");}
  sprintf(c1values[3], "%.1f", gps.speed.kmph());
  sprintf(c1values[4], "%.1f", gps.course.deg());
  if (gps.date.isValid()) {sprintf(c1values[5], "%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year() % 100);} else {sprintf(c1values[5], "NoData");}
  if (gps.time.isValid()) {sprintf(c1values[6], "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());} else {sprintf(c1values[6], "NoData");}
  // Fix: quality + mode (e.g. "GPS 3D" or "DGPS 2D").
  {
    const char* fixQ = "None";
    if (ggaFixQuality == 1) fixQ = "GPS";
    else if (ggaFixQuality == 2) fixQ = "DGPS";
    else if (ggaFixQuality == 4) fixQ = "RTK";
    else if (ggaFixQuality == 5) fixQ = "FRTK";
    const char* fixM = "";
    if (gsaFixMode == 2) fixM = " 2D";
    else if (gsaFixMode == 3) fixM = " 3D";
    sprintf(c1values[7], "%s%s", fixQ, fixM);
  }
  // Draw.
  for (int i = 0; i < 8; i++) {
    frameBuf.fillRect(x, y, 90, 16, TFT_BLACK);
    frameBuf.drawRect(x, y, 90, 16, TFT_DARKGREY);
    frameBuf.setTextColor(TFT_WHITE);
    frameBuf.setCursor(x + 4, y + 4);
    frameBuf.setTextSize(1);
    frameBuf.printf("%s: %s", c1labels[i], c1values[i]);
    y += 16;
  }
  // Column 2.
  y = 2;
  x += 89;
  // Labels.
  const char* c2labels[] = { "Vis", "Usd", "PD", "HD", "VD", "Gp/Gl", "Ga/Bd", "Qz" };
  // Values.
  char c2values[8][12];
  int totalVisible = 0;
  int totalUsed = 0;
  int gpsVisible = 0;
  int glonassVisible = 0;
  int galileoVisible = 0;
  int beidouVisible = 0;
  qzssVisible = 0;
  for (auto &sat : satellites) {
    if (sat.used) totalUsed++;
    if (sat.visible) {
      totalVisible++;
      if (sat.system == "GPS") gpsVisible++;
      else if (sat.system == "GLONASS") glonassVisible++;
      else if (sat.system == "Galileo") galileoVisible++;
      else if (sat.system == "BeiDou") beidouVisible++;
      else if (sat.system == "QZSS") qzssVisible++;
    }
  }
  sprintf(c2values[0], "%d/%d", totalVisible, (int)satellites.size());
  if (gps.satellites.isValid()) {sprintf(c2values[1], "%d", gps.satellites.value());} else {sprintf(c2values[1], "%d", totalUsed);}
  sprintf(c2values[2], "%.1f", pdop);
  sprintf(c2values[3], "%.1f", gps.hdop.hdop());
  sprintf(c2values[4], "%.1f", vdop);
  sprintf(c2values[5], "%d/%d", gpsVisible, glonassVisible);
  sprintf(c2values[6], "%d/%d", galileoVisible, beidouVisible);
  sprintf(c2values[7], "%d", qzssVisible);
  // Draw.
  for (int i = 0; i < 8; i++) {
    frameBuf.fillRect(x, y, 65, 16, TFT_BLACK);
    frameBuf.drawRect(x, y, 65, 16, TFT_DARKGREY);
    frameBuf.setTextColor(TFT_WHITE);
    frameBuf.setCursor(x + 2, y + 4);
    frameBuf.setTextSize(1);
    frameBuf.printf("%s:%s", c2labels[i], c2values[i]);
    y += 16;
  }
}

// ------------------------------------------------------------------
//  Screen 2: SCR_SKY_VIEW  (Enlarged satellite sky plot)
// ------------------------------------------------------------------
void drawScreenSkyView() {
  int plotSize = 125;
  int px = (screenW - plotSize) / 2;
  int py = (screenH - plotSize) / 2;
  drawSkyPlotAt(px, py, plotSize, plotSize, !hidePlotId, !hidePlotSystem);
}

// ------------------------------------------------------------------
//  Screen 3: SCR_SIGNAL_BARS  (Satellite SNR bar chart)
// ------------------------------------------------------------------
void drawScreenSignalBars() {
  int chartX = 22, chartY = 14;
  int chartW = 214, chartH = 100;
  int baseY = chartY + chartH;

  // Collect visible satellites.
  std::vector<SatData*> visSats;
  for (auto &sat : satellites) {
    if (sat.visible) visSats.push_back(&sat);
  }

  if (visSats.empty()) {
    frameBuf.setTextSize(1);
    frameBuf.setTextColor(TFT_DARKGREY);
    frameBuf.setCursor(60, 60);
    frameBuf.print("No satellites visible");
    return;
  }

  // Sort by system then ID.
  std::sort(visSats.begin(), visSats.end(), [](SatData* a, SatData* b) {
    if (a->system != b->system) return a->system < b->system;
    return a->id < b->id;
  });

  int numSats = visSats.size();
  int maxBars = (numSats > 30) ? 30 : numSats;
  int barW = (chartW - maxBars) / maxBars;
  if (barW < 3) barW = 3;
  if (barW > 10) barW = 10;
  int gap = 1;
  int totalBarW = maxBars * (barW + gap);
  int startX = chartX + (chartW - totalBarW) / 2;

  // Title line: SNR dB + satellite count.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_WHITE);
  char titleBuf[32];
  snprintf(titleBuf, sizeof(titleBuf), "SNR (dB)  %d sats", numSats);
  frameBuf.setCursor(chartX, 2);
  frameBuf.print(titleBuf);

  // Y-axis labels.
  frameBuf.setTextColor(TFT_DARKGREY);
  frameBuf.setCursor(2, chartY);
  frameBuf.print("50");
  frameBuf.setCursor(2, chartY + chartH / 2 - 4);
  frameBuf.print("25");
  frameBuf.setCursor(5, baseY - 8);
  frameBuf.print("0");

  // Horizontal ref lines.
  for (int v = 0; v <= 50; v += 25) {
    int ly = baseY - (v * chartH / 50);
    frameBuf.drawLine(chartX, ly, chartX + chartW, ly, TFT_DARKGREY);
  }

  // Draw bars.
  for (int i = 0; i < maxBars; i++) {
    SatData* sat = visSats[i];
    int snr = constrain(sat->snr, 0, 50);
    int barH = (snr * chartH) / 50;
    int bx = startX + i * (barW + gap);
    uint16_t color = systemColor(sat->system);
    // Bar outline for used satellites.
    if (sat->used && barH > 0)
      frameBuf.drawRect(bx - 1, baseY - barH - 1, barW + 2, barH + 2, TFT_WHITE);
    if (barH > 0)
      frameBuf.fillRect(bx, baseY - barH, barW, barH, color);
    // SNR value on top of bar (if bars are wide enough).
    if (barW >= 5 && snr > 0) {
      char snrBuf[4];
      snprintf(snrBuf, sizeof(snrBuf), "%d", snr);
      int tw = strlen(snrBuf) * 6;
      frameBuf.setTextSize(1);
      frameBuf.setTextColor(TFT_WHITE);
      frameBuf.setCursor(bx + (barW - tw) / 2, baseY - barH - 9);
      frameBuf.print(snrBuf);
    }
  }

  // Satellite ID labels below bars (rotated as vertical digits if narrow).
  frameBuf.setTextSize(1);
  for (int i = 0; i < maxBars; i++) {
    SatData* sat = visSats[i];
    int bx = startX + i * (barW + gap);
    uint16_t color = systemColor(sat->system);
    frameBuf.setTextColor(color);
    char idBuf[4];
    snprintf(idBuf, sizeof(idBuf), "%d", sat->id);
    if (barW >= 8) {
      // Horizontal ID below bar.
      frameBuf.setCursor(bx, baseY + 2);
      frameBuf.print(idBuf);
    } else if (barW >= 5) {
      // Just first 2 digits.
      frameBuf.setCursor(bx, baseY + 2);
      idBuf[2] = '\0';
      frameBuf.print(idBuf);
    }
  }

  // Constellation legend at bottom.
  int legY = 127;
  int legX = 4;
  frameBuf.setTextSize(1);
  struct { const char* label; uint16_t color; } legend[] = {
    {"GP", TFT_GREEN}, {"GL", TFT_RED}, {"GA", TFT_CYAN},
    {"BD", TFT_YELLOW}, {"QZ", TFT_MAGENTA}
  };
  for (int i = 0; i < 5; i++) {
    frameBuf.fillRect(legX, legY, 6, 6, legend[i].color);
    frameBuf.setTextColor(legend[i].color);
    frameBuf.setCursor(legX + 8, legY - 1);
    frameBuf.print(legend[i].label);
    legX += 8 + strlen(legend[i].label) * 6 + 6;
  }
  // Used indicator.
  frameBuf.setTextColor(TFT_WHITE);
  frameBuf.setCursor(legX + 4, legY - 1);
  frameBuf.print("[]=Used");
}

// ------------------------------------------------------------------
//  Screen 4: SCR_FIX_SUMMARY  (Fix quality and DOP details)
// ------------------------------------------------------------------
void drawScreenFixSummary() {
  frameBuf.setTextSize(1);
  int y = 4;
  int lh = 14;
  char buf[40];

  // Fix quality.
  const char* fixQ = "None";
  if (ggaFixQuality == 1) fixQ = "GPS";
  else if (ggaFixQuality == 2) fixQ = "DGPS";
  else if (ggaFixQuality == 4) fixQ = "RTK";
  else if (ggaFixQuality == 5) fixQ = "Float RTK";
  frameBuf.setTextColor(TFT_GREEN);
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Fix Quality: %s", fixQ);
  frameBuf.print(buf);
  y += lh;

  // Fix mode.
  const char* fixM = "No Fix";
  if (gsaFixMode == 2) fixM = "2D";
  else if (gsaFixMode == 3) fixM = "3D";
  frameBuf.setTextColor(TFT_WHITE);
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Fix Mode:    %s", fixM);
  frameBuf.print(buf);
  y += lh;

  // PDOP.
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "PDOP: %.1f (%s)", pdop, dopQuality(pdop));
  frameBuf.print(buf);
  y += lh;

  // HDOP.
  float hdopVal = gps.hdop.hdop();
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "HDOP: %.1f (%s)", hdopVal, dopQuality(hdopVal));
  frameBuf.print(buf);
  y += lh;

  // VDOP.
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "VDOP: %.1f (%s)", vdop, dopQuality(vdop));
  frameBuf.print(buf);
  y += lh;

  // Satellite counts.
  SatCounts sc;
  countSatellites(sc);
  frameBuf.setTextColor(TFT_LIGHTGREY);
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Used: %d  Visible: %d  Total: %d", sc.totalUsed, sc.totalVisible, sc.total);
  frameBuf.print(buf);
  y += lh;

  // Constellation counts.
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "GP:%d GL:%d GA:%d BD:%d QZ:%d",
    sc.gpsVis, sc.glonassVis, sc.galileoVis, sc.beidouVis, sc.qzssVis);
  frameBuf.print(buf);
  y += lh;

  // Geoid height.
  frameBuf.setCursor(4, y);
  if (geoidValid)
    snprintf(buf, sizeof(buf), "Geoid Height: %.1f m", geoidHeight);
  else
    snprintf(buf, sizeof(buf), "Geoid Height: N/A");
  frameBuf.print(buf);
  y += lh;

  // IMU info.
  if (imuAvailable) {
    frameBuf.setCursor(4, y);
    snprintf(buf, sizeof(buf), "IMU: P:%+.0f R:%+.0f G:%.1f T:%.0fC", imuPitch, imuRoll, imuGforce, imuTemp);
    frameBuf.print(buf);
  }
}

// ------------------------------------------------------------------
//  Screen 5: SCR_DASHBOARD  (Large speed + heading + altitude)
// ------------------------------------------------------------------
void drawScreenDashboard() {
  char buf[24];

  // Speed label.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_DARKGREY);
  int labelW = 7 * 6; // "SPEED" + "km/h" area
  frameBuf.setCursor((screenW - labelW) / 2, 6);
  frameBuf.print("km/h");

  // Speed value (large).
  frameBuf.setTextSize(3);
  frameBuf.setTextColor(TFT_GREEN);
  float spd = gps.speed.kmph();
  snprintf(buf, sizeof(buf), "%.1f", spd);
  int spdW = strlen(buf) * 18;
  frameBuf.setCursor((screenW - spdW) / 2, 18);
  frameBuf.print(buf);

  // Heading + cardinal.
  frameBuf.setTextSize(2);
  frameBuf.setTextColor(TFT_WHITE);
  if (gps.course.isValid()) {
    snprintf(buf, sizeof(buf), "HDG %.0f %s", gps.course.deg(), cardinalFromHeading(gps.course.deg()));
  } else {
    snprintf(buf, sizeof(buf), "HDG ---");
  }
  int hdgW = strlen(buf) * 12;
  frameBuf.setCursor((screenW - hdgW) / 2, 56);
  frameBuf.print(buf);

  // Altitude.
  frameBuf.setTextSize(2);
  frameBuf.setTextColor(TFT_CYAN);
  if (gps.altitude.isValid()) {
    snprintf(buf, sizeof(buf), "ALT %.0fm", gps.altitude.meters());
  } else {
    snprintf(buf, sizeof(buf), "ALT ---");
  }
  int altW = strlen(buf) * 12;
  frameBuf.setCursor((screenW - altW) / 2, 90);
  frameBuf.print(buf);

  // IMU pitch/roll/G.
  if (imuAvailable) {
    frameBuf.setTextSize(1);
    frameBuf.setTextColor(TFT_LIGHTGREY);
    snprintf(buf, sizeof(buf), "P:%+.0f R:%+.0f G:%.1f T:%.0fC", imuPitch, imuRoll, imuGforce, imuTemp);
    int prW = strlen(buf) * 6;
    frameBuf.setCursor((screenW - prW) / 2, 114);
    frameBuf.print(buf);
  }
}

// ------------------------------------------------------------------
//  Screen 6: SCR_COORDINATES  (Large lat/lon in decimal and DMS)
// ------------------------------------------------------------------
void drawScreenCoordinates() {
  char buf[40];
  int y = 6;

  if (!gps.location.isValid()) {
    frameBuf.setTextSize(2);
    frameBuf.setTextColor(TFT_DARKGREY);
    frameBuf.setCursor(40, 55);
    frameBuf.print("No Fix");
    return;
  }

  float lat = gps.location.lat();
  float lon = gps.location.lng();

  // Latitude decimal.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_DARKGREY);
  frameBuf.setCursor(4, y);
  frameBuf.print("LAT");
  y += 10;

  frameBuf.setTextSize(2);
  frameBuf.setTextColor(TFT_GREEN);
  snprintf(buf, sizeof(buf), "%.6f", lat);
  frameBuf.setCursor(4, y);
  frameBuf.print(buf);
  y += 18;

  // Latitude DMS.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_WHITE);
  float absLat = fabs(lat);
  int latD = (int)absLat;
  float latRem = (absLat - latD) * 60.0;
  int latM = (int)latRem;
  float latS = (latRem - latM) * 60.0;
  snprintf(buf, sizeof(buf), "%d%c %d' %.1f\" %s", latD, 0xF8, latM, latS, lat >= 0 ? "N" : "S");
  frameBuf.setCursor(4, y);
  frameBuf.print(buf);
  y += 14;

  // Longitude decimal.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_DARKGREY);
  frameBuf.setCursor(4, y);
  frameBuf.print("LON");
  y += 10;

  frameBuf.setTextSize(2);
  frameBuf.setTextColor(TFT_GREEN);
  snprintf(buf, sizeof(buf), "%.6f", lon);
  frameBuf.setCursor(4, y);
  frameBuf.print(buf);
  y += 18;

  // Longitude DMS.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_WHITE);
  float absLon = fabs(lon);
  int lonD = (int)absLon;
  float lonRem = (absLon - lonD) * 60.0;
  int lonM = (int)lonRem;
  float lonS = (lonRem - lonM) * 60.0;
  snprintf(buf, sizeof(buf), "%d%c %d' %.1f\" %s", lonD, 0xF8, lonM, lonS, lon >= 0 ? "E" : "W");
  frameBuf.setCursor(4, y);
  frameBuf.print(buf);
}

// ------------------------------------------------------------------
//  Screen 8: SCR_BREADCRUMB  (Track trail on local XY projection)
// ------------------------------------------------------------------
void drawScreenBreadcrumb() {
  int mapX = 4, mapY = 2, mapW = 232, mapH = 131;

  if (trackCount < 1) {
    frameBuf.setTextSize(1);
    frameBuf.setTextColor(TFT_DARKGREY);
    frameBuf.setCursor(60, 60);
    frameBuf.print("No track data yet");
    return;
  }

  // Latest point as center reference.
  int latestIdx = (trackHead - 1 + TRACK_MAX) % TRACK_MAX;
  float cLat = trackBuf[latestIdx].lat;
  float cLon = trackBuf[latestIdx].lon;
  float cosLat = cos(radians(cLat));

  // Convert to local XY (meters).
  float px[TRACK_MAX], py[TRACK_MAX];
  float maxDist = 1.0;
  for (int i = 0; i < trackCount; i++) {
    int idx = (trackHead - trackCount + i + TRACK_MAX) % TRACK_MAX;
    px[i] = (trackBuf[idx].lon - cLon) * cosLat * 111320.0;
    py[i] = (trackBuf[idx].lat - cLat) * 111320.0;
    float d = max(fabs(px[i]), fabs(py[i]));
    if (d > maxDist) maxDist = d;
  }

  // Auto-scale with margin.
  float halfMap = min((float)mapW, (float)mapH) / 2.0;
  float scale = halfMap / (maxDist * 1.2);
  int mcx = mapX + mapW / 2;
  int mcy = mapY + mapH / 2;

  // Border.
  frameBuf.drawRect(mapX, mapY, mapW, mapH, TFT_DARKGREY);

  // Track line.
  for (int i = 1; i < trackCount; i++) {
    int sx1 = mcx + (int)(px[i-1] * scale);
    int sy1 = mcy - (int)(py[i-1] * scale);
    int sx2 = mcx + (int)(px[i] * scale);
    int sy2 = mcy - (int)(py[i] * scale);
    sx1 = constrain(sx1, mapX + 1, mapX + mapW - 2);
    sy1 = constrain(sy1, mapY + 1, mapY + mapH - 2);
    sx2 = constrain(sx2, mapX + 1, mapX + mapW - 2);
    sy2 = constrain(sy2, mapY + 1, mapY + mapH - 2);
    // Fade older segments.
    uint16_t col = (i > trackCount / 2) ? TFT_GREEN : TFT_DARKGREY;
    frameBuf.drawLine(sx1, sy1, sx2, sy2, col);
  }

  // Current position dot.
  frameBuf.fillCircle(mcx, mcy, 3, TFT_RED);

  // North arrow.
  frameBuf.drawLine(mapX + mapW - 12, mapY + 16, mapX + mapW - 12, mapY + 5, TFT_WHITE);
  frameBuf.drawLine(mapX + mapW - 12, mapY + 5, mapX + mapW - 14, mapY + 9, TFT_WHITE);
  frameBuf.drawLine(mapX + mapW - 12, mapY + 5, mapX + mapW - 10, mapY + 9, TFT_WHITE);
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_WHITE);
  frameBuf.setCursor(mapX + mapW - 15, mapY + 17);
  frameBuf.print("N");
}

// ------------------------------------------------------------------
//  Screen 9: SCR_ALT_PROFILE  (Altitude vs time/distance graph)
// ------------------------------------------------------------------
void drawScreenAltProfile() {
  int gx = 30, gy = 14, gw = 205, gh = 112;

  if (trackCount < 2) {
    frameBuf.setTextSize(1);
    frameBuf.setTextColor(TFT_DARKGREY);
    frameBuf.setCursor(60, 60);
    frameBuf.print("Collecting altitude data...");
    return;
  }

  // Extract altitude values.
  float vals[TRACK_MAX];
  float minV = 99999, maxV = -99999, curV = 0;
  for (int i = 0; i < trackCount; i++) {
    int idx = (trackHead - trackCount + i + TRACK_MAX) % TRACK_MAX;
    vals[i] = trackBuf[idx].altM;
    if (vals[i] < minV) minV = vals[i];
    if (vals[i] > maxV) maxV = vals[i];
  }
  curV = vals[trackCount - 1];
  float range = maxV - minV;
  if (range < 1.0) { range = 1.0; minV -= 0.5; maxV += 0.5; }

  // Axes.
  frameBuf.drawLine(gx, gy, gx, gy + gh, TFT_DARKGREY);
  frameBuf.drawLine(gx, gy + gh, gx + gw, gy + gh, TFT_DARKGREY);

  // Y-axis labels.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_DARKGREY);
  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", maxV);
  frameBuf.setCursor(2, gy);
  frameBuf.print(buf);
  snprintf(buf, sizeof(buf), "%.0f", minV);
  frameBuf.setCursor(2, gy + gh - 8);
  frameBuf.print(buf);

  // Plot line.
  for (int i = 1; i < trackCount; i++) {
    int x1 = gx + (i - 1) * gw / (trackCount - 1);
    int y1 = gy + gh - (int)((vals[i-1] - minV) / range * gh);
    int x2 = gx + i * gw / (trackCount - 1);
    int y2 = gy + gh - (int)((vals[i] - minV) / range * gh);
    frameBuf.drawLine(x1, y1, x2, y2, TFT_CYAN);
  }

  // Current/max/min labels.
  frameBuf.setTextColor(TFT_CYAN);
  snprintf(buf, sizeof(buf), "Now:%.0fm", curV);
  frameBuf.setCursor(gx + gw - 60, gy - 4);
  frameBuf.print(buf);

  frameBuf.setTextColor(TFT_WHITE);
  frameBuf.setCursor(gx + 40, gy - 4);
  snprintf(buf, sizeof(buf), "Alt(m)");
  frameBuf.print(buf);
}

// ------------------------------------------------------------------
//  Screen 10: SCR_SPEED_GRAPH  (Speed vs time graph)
// ------------------------------------------------------------------
void drawScreenSpeedGraph() {
  int gx = 30, gy = 14, gw = 205, gh = 112;

  if (trackCount < 2) {
    frameBuf.setTextSize(1);
    frameBuf.setTextColor(TFT_DARKGREY);
    frameBuf.setCursor(60, 60);
    frameBuf.print("Collecting speed data...");
    return;
  }

  // Extract speed values.
  float vals[TRACK_MAX];
  float minV = 99999, maxV = -99999, curV = 0, sumV = 0;
  for (int i = 0; i < trackCount; i++) {
    int idx = (trackHead - trackCount + i + TRACK_MAX) % TRACK_MAX;
    vals[i] = trackBuf[idx].speedKmph;
    if (vals[i] < minV) minV = vals[i];
    if (vals[i] > maxV) maxV = vals[i];
    sumV += vals[i];
  }
  curV = vals[trackCount - 1];
  float avgV = sumV / trackCount;
  if (minV > 0) minV = 0; // Speed graph starts at 0.
  float range = maxV - minV;
  if (range < 1.0) { range = 1.0; maxV = minV + 1.0; }

  // Axes.
  frameBuf.drawLine(gx, gy, gx, gy + gh, TFT_DARKGREY);
  frameBuf.drawLine(gx, gy + gh, gx + gw, gy + gh, TFT_DARKGREY);

  // Y-axis labels.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_DARKGREY);
  char buf[16];
  snprintf(buf, sizeof(buf), "%.0f", maxV);
  frameBuf.setCursor(2, gy);
  frameBuf.print(buf);
  snprintf(buf, sizeof(buf), "%.0f", minV);
  frameBuf.setCursor(5, gy + gh - 8);
  frameBuf.print(buf);

  // Plot line.
  for (int i = 1; i < trackCount; i++) {
    int x1 = gx + (i - 1) * gw / (trackCount - 1);
    int y1 = gy + gh - (int)((vals[i-1] - minV) / range * gh);
    int x2 = gx + i * gw / (trackCount - 1);
    int y2 = gy + gh - (int)((vals[i] - minV) / range * gh);
    frameBuf.drawLine(x1, y1, x2, y2, TFT_YELLOW);
  }

  // Labels.
  frameBuf.setTextColor(TFT_YELLOW);
  snprintf(buf, sizeof(buf), "Now:%.1f", curV);
  frameBuf.setCursor(gx + gw - 60, gy - 4);
  frameBuf.print(buf);

  frameBuf.setTextColor(TFT_WHITE);
  frameBuf.setCursor(gx + 40, gy - 4);
  snprintf(buf, sizeof(buf), "Spd(km/h) avg:%.1f", avgV);
  frameBuf.print(buf);
}

// ------------------------------------------------------------------
//  Screen 11: SCR_TRIP_STATS  (Trip distance, time, averages)
// ------------------------------------------------------------------
void drawScreenTripStats() {
  frameBuf.setTextSize(1);
  int y = 4;
  int lh = 14;
  char buf[40];

  if (!trip.hasPrev) {
    frameBuf.setTextColor(TFT_DARKGREY);
    frameBuf.setCursor(50, 60);
    frameBuf.print("No trip data yet");
    return;
  }

  uint32_t elapsed = millis() - trip.startMillis;
  float avgSpd = (trip.movingMillis > 1000) ? (trip.totalDistKm / (trip.movingMillis / 3600000.0)) : 0;
  char elBuf[12], mvBuf[12];
  formatDuration(elapsed, elBuf, sizeof(elBuf));
  formatDuration(trip.movingMillis, mvBuf, sizeof(mvBuf));

  frameBuf.setTextColor(TFT_GREEN);
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Distance:  %.2f km", trip.totalDistKm);
  frameBuf.print(buf);
  y += lh;

  frameBuf.setTextColor(TFT_WHITE);
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Elapsed:   %s", elBuf);
  frameBuf.print(buf);
  y += lh;

  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Moving:    %s", mvBuf);
  frameBuf.print(buf);
  y += lh;

  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Avg Speed: %.1f km/h", avgSpd);
  frameBuf.print(buf);
  y += lh;

  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Max Speed: %.1f km/h", trip.maxSpeedKmph);
  frameBuf.print(buf);
  y += lh;

  frameBuf.setTextColor(TFT_CYAN);
  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Alt Gain:  +%.0f m", trip.totalAscentM);
  frameBuf.print(buf);
  y += lh;

  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Alt Loss:  -%.0f m", trip.totalDescentM);
  frameBuf.print(buf);
  y += lh;

  frameBuf.setCursor(4, y);
  snprintf(buf, sizeof(buf), "Alt Range: %.0f - %.0f m", trip.minAltM, trip.maxAltM);
  frameBuf.print(buf);
  y += lh;

  // Max G-force (from IMU).
  if (imuAvailable) {
    frameBuf.setTextColor(TFT_YELLOW);
    frameBuf.setCursor(4, y);
    snprintf(buf, sizeof(buf), "Max G:     %.2f", tripMaxG);
    frameBuf.print(buf);
  }
}

// ------------------------------------------------------------------
//  Screen 12: SCR_CONSTELLATION  (Per-system satellite breakdown)
// ------------------------------------------------------------------
void drawScreenConstellation() {
  SatCounts sc;
  countSatellites(sc);

  frameBuf.setTextSize(1);
  int y = 4;
  int lh = 17;
  char buf[40];

  // Header.
  frameBuf.setTextColor(TFT_DARKGREY);
  frameBuf.setCursor(4, y);
  frameBuf.print("System     Visible  Used");
  y += lh;
  frameBuf.drawLine(4, y - 2, 200, y - 2, TFT_DARKGREY);

  // GPS.
  frameBuf.fillCircle(10, y + 3, 3, TFT_GREEN);
  frameBuf.setTextColor(TFT_WHITE);
  frameBuf.setCursor(18, y);
  snprintf(buf, sizeof(buf), "GPS         %3d     %3d", sc.gpsVis, sc.gpsUsed);
  frameBuf.print(buf);
  y += lh;

  // GLONASS.
  frameBuf.fillCircle(10, y + 3, 3, TFT_RED);
  frameBuf.setCursor(18, y);
  snprintf(buf, sizeof(buf), "GLONASS     %3d     %3d", sc.glonassVis, sc.glonassUsed);
  frameBuf.print(buf);
  y += lh;

  // Galileo.
  frameBuf.fillCircle(10, y + 3, 3, TFT_CYAN);
  frameBuf.setCursor(18, y);
  snprintf(buf, sizeof(buf), "Galileo     %3d     %3d", sc.galileoVis, sc.galileoUsed);
  frameBuf.print(buf);
  y += lh;

  // BeiDou.
  frameBuf.fillCircle(10, y + 3, 3, TFT_YELLOW);
  frameBuf.setCursor(18, y);
  snprintf(buf, sizeof(buf), "BeiDou      %3d     %3d", sc.beidouVis, sc.beidouUsed);
  frameBuf.print(buf);
  y += lh;

  // QZSS.
  frameBuf.fillCircle(10, y + 3, 3, TFT_MAGENTA);
  frameBuf.setCursor(18, y);
  snprintf(buf, sizeof(buf), "QZSS        %3d     %3d", sc.qzssVis, sc.qzssUsed);
  frameBuf.print(buf);
  y += lh;

  // Divider.
  frameBuf.drawLine(4, y - 2, 200, y - 2, TFT_DARKGREY);

  // Total.
  frameBuf.setTextColor(TFT_GREEN);
  frameBuf.setCursor(18, y);
  snprintf(buf, sizeof(buf), "Total       %3d     %3d", sc.totalVisible, sc.totalUsed);
  frameBuf.print(buf);
}

// ------------------------------------------------------------------
//  Screen 13: SCR_NMEA_MONITOR  (Raw NMEA sentence scrollback)
// ------------------------------------------------------------------
void drawScreenNmeaMonitor() {
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_GREEN, TFT_BLACK);
  int y = 1;
  // 135px / 8px per line = 16 lines, tight fit using full screen.
  int maxLines = (nmeaBufCount < 16) ? nmeaBufCount : 16;

  if (maxLines == 0) {
    frameBuf.setTextColor(TFT_DARKGREY);
    frameBuf.setCursor(40, 60);
    frameBuf.print("Waiting for NMEA data...");
    return;
  }

  for (int i = 0; i < maxLines; i++) {
    int idx = (nmeaBufHead - maxLines + i + NMEA_BUF_LINES) % NMEA_BUF_LINES;
    frameBuf.setCursor(2, y);
    // Truncate to fit screen (40 chars at 6px each = 240px).
    char truncated[41];
    strncpy(truncated, nmeaBuf[idx], 40);
    truncated[40] = '\0';
    frameBuf.print(truncated);
    y += 8;
  }
}

// ------------------------------------------------------------------
//  DST helpers for clock
// ------------------------------------------------------------------
// Day of week: 0=Sun .. 6=Sat  (Tomohiko Sakamoto)
static int calcDow(int y, int m, int d) {
  static const int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
  if (m < 3) y--;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}
// Day-of-month of the Nth weekday (wday 0=Sun) in given month
static int nthWday(int y, int m, int wday, int n) {
  int d1 = calcDow(y, m, 1);
  return 1 + ((wday - d1 + 7) % 7) + (n - 1) * 7;
}
// Day-of-month of last weekday in given month
static int lastWday(int y, int m, int wday) {
  static const int dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int days = dim[m-1];
  if (m == 2 && (y%4==0 && (y%100!=0 || y%400==0))) days = 29;
  int dl = calcDow(y, m, days);
  return days - ((dl - wday + 7) % 7);
}
// Get DST offset (0 or 1 hour) based on GPS position and UTC date/time.
// Covers: US/Canada, EU, southern Australia, New Zealand.
static int getDST(int y, int mon, int d, int utcH, float lat, float lon) {
  // --- North America (US/Canada): 2nd Sun Mar → 1st Sun Nov ---
  if (lat > 24 && lat < 72 && lon > -140 && lon < -50) {
    // Hawaii: no DST
    if (lat < 23 && lon < -154) return 0;
    // Arizona (excl. Navajo): no DST
    if (lat > 31 && lat < 37.5f && lon > -115 && lon < -109) return 0;
    int sD = nthWday(y, 3, 0, 2);   // 2nd Sunday March
    int eD = nthWday(y, 11, 0, 1);  // 1st Sunday November
    // Compare in local standard time
    int stdOff = (int)roundf(lon / 15.0f);
    int lh = utcH + stdOff;
    int ld = d;
    if (lh >= 24) { lh -= 24; ld++; } else if (lh < 0) { lh += 24; ld--; }
    if (mon > 3 && mon < 11) return 1;
    if (mon < 3 || mon > 11) return 0;
    if (mon == 3) return (ld > sD || (ld == sD && lh >= 2)) ? 1 : 0;
    if (mon == 11) return (ld < eD || (ld == eD && lh < 1)) ? 1 : 0;
    return 0;
  }
  // --- Europe (EU): last Sun Mar → last Sun Oct, at 01:00 UTC ---
  if (lat > 34 && lat < 72 && lon > -12 && lon < 45) {
    int sD = lastWday(y, 3, 0);
    int eD = lastWday(y, 10, 0);
    if (mon > 3 && mon < 10) return 1;
    if (mon < 3 || mon > 10) return 0;
    if (mon == 3) return (d > sD || (d == sD && utcH >= 1)) ? 1 : 0;
    if (mon == 10) return (d < eD || (d == eD && utcH < 1)) ? 1 : 0;
    return 0;
  }
  // --- Southern Australia (NSW,VIC,SA,TAS,ACT): 1st Sun Oct → 1st Sun Apr ---
  if (lat < -28 && lon > 138 && lon < 155) {
    int sD = nthWday(y, 10, 0, 1);
    int eD = nthWday(y, 4, 0, 1);
    int lh = utcH + 10; int ld = d; // AEST = UTC+10
    if (lh >= 24) { lh -= 24; ld++; }
    if (mon > 10 || mon < 4) return 1;
    if (mon > 4 && mon < 10) return 0;
    if (mon == 10) return (ld > sD || (ld == sD && lh >= 2)) ? 1 : 0;
    if (mon == 4) return (ld < eD || (ld == eD && lh < 2)) ? 1 : 0;
    return 0;
  }
  // --- New Zealand: last Sun Sep → 1st Sun Apr ---
  if (lat < -34 && lon > 165) {
    int sD = lastWday(y, 9, 0);
    int eD = nthWday(y, 4, 0, 1);
    int lh = utcH + 12; int ld = d;
    if (lh >= 24) { lh -= 24; ld++; }
    if (mon > 9 || mon < 4) return 1;
    if (mon > 4 && mon < 9) return 0;
    if (mon == 9) return (ld > sD || (ld == sD && lh >= 2)) ? 1 : 0;
    if (mon == 4) return (ld < eD || (ld == eD && lh < 2)) ? 1 : 0;
    return 0;
  }
  return 0; // No DST elsewhere
}

// ------------------------------------------------------------------
//  Screen 14: SCR_GPS_CLOCK  (Local clock display)
// ------------------------------------------------------------------
void drawScreenGpsClock() {
  char buf[32];

  if (!gps.time.isValid()) {
    frameBuf.setTextSize(2);
    frameBuf.setTextColor(TFT_DARKGREY);
    frameBuf.setCursor(40, 55);
    frameBuf.print("No Time Data");
    return;
  }

  int utcH = gps.time.hour();
  int utcM = gps.time.minute();
  int utcS = gps.time.second();

  // Compute timezone offset from longitude (lon/15 rounded) + DST.
  bool hasLoc = gps.location.isValid();
  int stdOff = 0;  // standard timezone offset
  int dstAdj = 0;  // DST adjustment (0 or 1)
  if (hasLoc) {
    stdOff = (int)roundf(gps.location.lng() / 15.0f);
    if (gps.date.isValid())
      dstAdj = getDST(gps.date.year(), gps.date.month(), gps.date.day(),
                       utcH, gps.location.lat(), gps.location.lng());
  }
  int tzOff = stdOff + dstAdj;

  // Compute local time using mktime for proper date rollover.
  int localH = utcH, localMin = utcM, localSec = utcS;
  int localY = 0, localMon = 0, localD = 0;
  bool localDateValid = false;
  if (gps.date.isValid()) {
    struct tm utcTm = {};
    utcTm.tm_year = gps.date.year() - 1900;
    utcTm.tm_mon  = gps.date.month() - 1;
    utcTm.tm_mday = gps.date.day();
    utcTm.tm_hour = utcH;
    utcTm.tm_min  = utcM;
    utcTm.tm_sec  = utcS;
    time_t epoch = mktime(&utcTm);
    epoch += (long)tzOff * 3600L;
    struct tm localTm;
    gmtime_r(&epoch, &localTm);
    localH   = localTm.tm_hour;
    localMin = localTm.tm_min;
    localSec = localTm.tm_sec;
    localY   = localTm.tm_year + 1900;
    localMon = localTm.tm_mon + 1;
    localD   = localTm.tm_mday;
    localDateValid = true;
  } else {
    // No date: just offset hours
    localH = utcH + tzOff;
    if (localH >= 24) localH -= 24;
    else if (localH < 0) localH += 24;
  }

  // Large LOCAL time.
  frameBuf.setTextSize(3);
  frameBuf.setTextColor(TFT_GREEN);
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", localH, localMin, localSec);
  int timeW = strlen(buf) * 18;
  frameBuf.setCursor((screenW - timeW) / 2, 4);
  frameBuf.print(buf);

  // Local date.
  frameBuf.setTextSize(2);
  frameBuf.setTextColor(TFT_WHITE);
  if (localDateValid) {
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d", localY, localMon, localD);
  } else {
    snprintf(buf, sizeof(buf), "----/--/--");
  }
  int dateW = strlen(buf) * 12;
  frameBuf.setCursor((screenW - dateW) / 2, 30);
  frameBuf.print(buf);

  // Day of week + UTC offset.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_LIGHTGREY);
  if (localDateValid) {
    int y2 = localY, m2 = localMon, d2 = localD;
    if (m2 < 3) { m2 += 12; y2--; }
    int dow = (d2 + (13*(m2+1))/5 + y2 + y2/4 - y2/100 + y2/400) % 7;
    const char* days[] = {"Sat", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri"};
    if (hasLoc)
      snprintf(buf, sizeof(buf), "%s  UTC%+d%s", days[dow], tzOff, dstAdj ? " DST" : "");
    else
      snprintf(buf, sizeof(buf), "%s  UTC", days[dow]);
  } else {
    if (hasLoc)
      snprintf(buf, sizeof(buf), "UTC%+d%s", tzOff, dstAdj ? " DST" : "");
    else
      snprintf(buf, sizeof(buf), "UTC");
  }
  int dowW = strlen(buf) * 6;
  frameBuf.setCursor((screenW - dowW) / 2, 50);
  frameBuf.print(buf);

  // UTC reference time (smaller).
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_DARKGREY);
  snprintf(buf, sizeof(buf), "UTC %02d:%02d:%02d", utcH, utcM, utcS);
  int utcW = strlen(buf) * 6;
  frameBuf.setCursor((screenW - utcW) / 2, 64);
  frameBuf.print(buf);

  // Fix status indicator.
  frameBuf.setTextSize(1);
  const char* fixStr = "No Fix";
  uint16_t fixCol = TFT_RED;
  if (ggaFixQuality >= 1) {
    fixStr = "Fix OK";
    fixCol = TFT_GREEN;
  }
  frameBuf.setTextColor(fixCol);
  snprintf(buf, sizeof(buf), "GPS: %s", fixStr);
  int fixW = strlen(buf) * 6;
  frameBuf.setCursor((screenW - fixW) / 2, 80);
  frameBuf.print(buf);

  // Fix age.
  frameBuf.setTextColor(TFT_DARKGREY);
  uint32_t age = gps.location.age();
  if (age < 10000)
    snprintf(buf, sizeof(buf), "Age: %.1fs", age / 1000.0);
  else
    snprintf(buf, sizeof(buf), "Age: >10s");
  int ageW = strlen(buf) * 6;
  frameBuf.setCursor((screenW - ageW) / 2, 96);
  frameBuf.print(buf);

  // Temperature from IMU.
  if (imuAvailable) {
    frameBuf.setTextColor(TFT_DARKGREY);
    snprintf(buf, sizeof(buf), "%.1fC", imuTemp);
    int tmpW = strlen(buf) * 6;
    frameBuf.setCursor((screenW - tmpW) / 2, 112);
    frameBuf.print(buf);
  }
}

// ------------------------------------------------------------------
//  Screen 14: SCR_GPS_MAP  (World map with GPS position)
// ------------------------------------------------------------------

// Map data: auto-generated from Natural Earth 10m/50m datasets.
// Contains coastlines, borders, states, rivers, lakes, and 500 cities.
// ~2MB of PROGMEM data with bounding-box headers for fast rendering.
#include "map_data.h"

// Coordinate scale factor (degrees * COORD_SCALE = stored int16_t).
static const float COORD_INV = 1.0f / COORD_SCALE;

// Render a polyline array with bbox skip optimization.
// Format: 0x7FFE,minLat,maxLat,minLon,maxLon, ...points..., 0x7FFF, ...
// Two consecutive 0x7FFF = end.
void renderPolyArray(const int16_t* data, int totalVals, uint16_t color,
                     float latMin, float latMax, float lonMin, float lonMax,
                     float latRange, float lonRange) {
  int idx = 0;
  int prevSX = -9999, prevSY = -9999;
  bool segActive = false;
  while (idx < totalVals) {
    int16_t v = (int16_t)pgm_read_word(&data[idx]);

    if (v == 0x7FFF) {
      idx++;
      if (idx >= totalVals) break;
      if ((int16_t)pgm_read_word(&data[idx]) == 0x7FFF) break;
      segActive = false;
      continue;
    }

    if (v == 0x7FFE) {
      // Bounding box header: minLat, maxLat, minLon, maxLon
      idx++;
      if (idx + 4 > totalVals) break;
      float bbMinLat = (int16_t)pgm_read_word(&data[idx])   * COORD_INV;
      float bbMaxLat = (int16_t)pgm_read_word(&data[idx+1]) * COORD_INV;
      float bbMinLon = (int16_t)pgm_read_word(&data[idx+2]) * COORD_INV;
      float bbMaxLon = (int16_t)pgm_read_word(&data[idx+3]) * COORD_INV;
      idx += 4;

      // Skip segment if bbox doesn't intersect viewport
      if (bbMaxLat < latMin || bbMinLat > latMax ||
          bbMaxLon < lonMin || bbMinLon > lonMax) {
        // Fast-forward to next 0x7FFF
        while (idx < totalVals) {
          if ((int16_t)pgm_read_word(&data[idx]) == 0x7FFF) {
            idx++;
            break;
          }
          idx++;
        }
        segActive = false;
        continue;
      }
      segActive = false;
      continue;
    }

    // Regular coordinate pair
    int16_t latS = v;
    idx++;
    if (idx >= totalVals) break;
    int16_t lonS = (int16_t)pgm_read_word(&data[idx]);
    idx++;
    float lat = latS * COORD_INV;
    float lon = lonS * COORD_INV;
    int sx = (int)((lon - lonMin) / lonRange * screenW);
    int sy = (int)((latMax - lat) / latRange * screenH);
    if (segActive) {
      if ((sx > -50 && sx < screenW + 50 && sy > -50 && sy < screenH + 50) ||
          (prevSX > -50 && prevSX < screenW + 50 && prevSY > -50 && prevSY < screenH + 50)) {
        frameBuf.drawLine(prevSX, prevSY, sx, sy, color);
      }
    }
    prevSX = sx;
    prevSY = sy;
    segActive = true;
  }
}

void drawScreenGpsMap() {
  // Center on GPS if available.
  float cLat = 0, cLon = 0;
  bool hasFix = gps.location.isValid();
  if (hasFix) {
    cLat = gps.location.lat();
    cLon = gps.location.lng();
  }

  // Visible range based on zoom.
  static const float zoomLon[] = {360, 180, 90, 45, 22.5, 11.25};
  static const float zoomLat[] = {180, 90, 45, 22.5, 11.25, 5.625};
  int z = constrain(mapZoom, 0, 5);
  float lonRange = zoomLon[z];
  float latRange = zoomLat[z];
  float latMin, latMax, lonMin, lonMax;
  if (z == 0) {
    latMin = -90; latMax = 90; lonMin = -180; lonMax = 180;
  } else {
    latMin = cLat - latRange / 2;
    latMax = cLat + latRange / 2;
    lonMin = cLon - lonRange / 2;
    lonMax = cLon + lonRange / 2;
  }

  // Grid lines.
  float gridStep;
  if (lonRange > 180) gridStep = 30;
  else if (lonRange > 90) gridStep = 15;
  else if (lonRange > 45) gridStep = 10;
  else gridStep = 5;
  for (float lat = floor(latMin / gridStep) * gridStep; lat <= latMax; lat += gridStep) {
    int sy = (int)((latMax - lat) / latRange * screenH);
    if (sy >= 0 && sy < screenH)
      frameBuf.drawLine(0, sy, screenW - 1, sy, 0x2104);
  }
  for (float lon = floor(lonMin / gridStep) * gridStep; lon <= lonMax; lon += gridStep) {
    int sx = (int)((lon - lonMin) / lonRange * screenW);
    if (sx >= 0 && sx < screenW)
      frameBuf.drawLine(sx, 0, sx, screenH - 1, 0x2104);
  }
  // Equator & prime meridian (brighter).
  int eqY = (int)((latMax - 0) / latRange * screenH);
  if (eqY >= 0 && eqY < screenH)
    frameBuf.drawLine(0, eqY, screenW - 1, eqY, TFT_DARKGREY);
  int pmX = (int)((0 - lonMin) / lonRange * screenW);
  if (pmX >= 0 && pmX < screenW)
    frameBuf.drawLine(pmX, 0, pmX, screenH - 1, TFT_DARKGREY);

  // Draw map layers from PROGMEM (back to front).
  // Lakes (filled appearance - cyan outlines)
  renderPolyArray(lakeData, sizeof(lakeData)/sizeof(int16_t), 0x0B5E,
                  latMin, latMax, lonMin, lonMax, latRange, lonRange);
  // Rivers (blue lines)
  renderPolyArray(riverData, sizeof(riverData)/sizeof(int16_t), 0x2B5F,
                  latMin, latMax, lonMin, lonMax, latRange, lonRange);
  // State/province outlines (dim cyan)
  renderPolyArray(stateData, sizeof(stateData)/sizeof(int16_t), 0x3BCD,
                  latMin, latMax, lonMin, lonMax, latRange, lonRange);
  // Country borders (dim yellow)
  renderPolyArray(borderData, sizeof(borderData)/sizeof(int16_t), 0x4A49,
                  latMin, latMax, lonMin, lonMax, latRange, lonRange);
  // Coastlines (bright green - on top)
  renderPolyArray(coastData, sizeof(coastData)/sizeof(int16_t), TFT_GREEN,
                  latMin, latMax, lonMin, lonMax, latRange, lonRange);

  // Draw cities.
  frameBuf.setTextSize(1);
  for (int i = 0; i < NUM_CITIES; i++) {
    MapCity city;
    memcpy_P(&city, &cityList[i], sizeof(MapCity));
    float clat = city.lat10 * COORD_INV;
    float clon = city.lon10 * COORD_INV;
    int cx = (int)((clon - lonMin) / lonRange * screenW);
    int cy = (int)((latMax - clat) / latRange * screenH);
    if (cx >= 1 && cx < screenW - 1 && cy >= 1 && cy < screenH - 1) {
      frameBuf.fillCircle(cx, cy, 1, TFT_YELLOW);
      if (z >= 4) {
        frameBuf.setTextColor(TFT_WHITE);
        int tx = cx + 3;
        if (tx + (int)strlen(city.name) * 6 > screenW)
          tx = cx - (int)strlen(city.name) * 6 - 2;
        frameBuf.setCursor(tx, cy - 3);
        frameBuf.print(city.name);
      }
    }
  }

  // GPS position marker.
  if (hasFix) {
    int gx = (int)((cLon - lonMin) / lonRange * screenW);
    int gy = (int)((latMax - cLat) / latRange * screenH);
    frameBuf.drawLine(gx - 6, gy, gx + 6, gy, TFT_RED);
    frameBuf.drawLine(gx, gy - 6, gx, gy + 6, TFT_RED);
    frameBuf.fillCircle(gx, gy, 2, TFT_RED);
  }

  // Zoom indicator.
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_DARKGREY);
  char zBuf[20];
  snprintf(zBuf, sizeof(zBuf), "z%d [z+/x-]", z);
  frameBuf.setCursor(2, screenH - 10);
  frameBuf.print(zBuf);

  // Position info.
  if (hasFix) {
    char posBuf[20];
    snprintf(posBuf, sizeof(posBuf), "%.2f,%.2f", cLat, cLon);
    int pw = strlen(posBuf) * 6;
    frameBuf.setCursor(screenW - pw - 2, screenH - 10);
    frameBuf.print(posBuf);
  } else {
    frameBuf.setCursor(screenW - 42, screenH - 10);
    frameBuf.print("No Fix");
  }
}

// ------------------------------------------------------------------
//  Screen 15: SCR_3D_GLOBE  (3D globe with satellites)
// ------------------------------------------------------------------

// Fast sin/cos lookup — replaces sinf/cosf for map points (~10x faster).
// Input: centidegrees (COORD_SCALE=100, e.g. 4523 = 45.23°).
static float _gSin[91];  // sin(0°)..sin(90°) — 364 bytes RAM
static bool  _gSinReady = false;

static float qsin(int cd) {
  // Normalise to 0..35999
  cd = ((cd % 36000) + 36000) % 36000;
  bool neg = false;
  if (cd >= 18000) { cd -= 18000; neg = true; }
  if (cd > 9000)   cd = 18000 - cd;
  int d = cd / 100, f = cd % 100;
  float s = _gSin[d];
  if (f && d < 90) s += (_gSin[d + 1] - s) * f * 0.01f;
  return neg ? -s : s;
}
static float qcos(int cd) { return qsin(cd + 9000); }

// Inline projection: centideg lat/lon → screen.  z>0 = visible.
#define GLOBE_PROJ(latCD, lonCD, r, sx_, sy_, z_) do { \
  float sl_ = qsin(latCD), cl_ = qcos(latCD);          \
  float sn_ = qsin(lonCD), cn_ = qcos(lonCD);          \
  float x_ = (r)*cl_*sn_, y_ = (r)*sl_, zz_ = (r)*cl_*cn_; \
  float x2_ = x_*cc + zz_*sc;                          \
  float z2_ = -x_*sc + zz_*cc;                         \
  (y_) = y_; /* keep y_ */                              \
  float y2_ = y_*ct - z2_*st;                           \
  (z_) = y_*st + z2_*ct;                                \
  (sx_) = gcx + (int)(x2_ * ER);                       \
  (sy_) = gcy - (int)(y2_ * ER);                       \
} while(0)

// Render one PROGMEM polyline array as globe coastlines with limb shading.
static void globeRenderLayer(const int16_t* data, int total,
    float sc, float cc, float st, float ct,
    int gcx, int gcy, float ER, int camLon100,
    uint16_t colBright, uint16_t colMid, uint16_t colDim) {
  int idx = 0;
  bool segActive = false;
  int psx = 0, psy = 0;
  float pz = -1;

  while (idx < total) {
    int16_t v = (int16_t)pgm_read_word(&data[idx]);
    if (v == 0x7FFF) {
      idx++;
      if (idx >= total) break;
      if ((int16_t)pgm_read_word(&data[idx]) == 0x7FFF) break;
      segActive = false;
      continue;
    }
    if (v == 0x7FFE) {
      idx++;
      if (idx + 4 > total) break;
      int16_t bLonMin = (int16_t)pgm_read_word(&data[idx+2]);
      int16_t bLonMax = (int16_t)pgm_read_word(&data[idx+3]);
      idx += 4;
      // Fast longitude-range cull (integer only — no trig)
      // Camera looks at -camLon (rotation sends lon+camLon → z-axis)
      int mid = (bLonMin + bLonMax) / 2;
      int span = (bLonMax - bLonMin) / 2;
      int rel = mid + camLon100;
      if (rel >  18000) rel -= 36000;
      if (rel < -18000) rel += 36000;
      if (rel < 0) rel = -rel;
      if (rel - span > 9500) { // >95° from camera centre → cull
        while (idx < total) {
          if ((int16_t)pgm_read_word(&data[idx]) == 0x7FFF) { idx++; break; }
          idx++;
        }
        segActive = false;
        continue;
      }
      segActive = false;
      continue;
    }
    int16_t latS = v;
    idx++;
    if (idx >= total) break;
    int16_t lonS = (int16_t)pgm_read_word(&data[idx]);
    idx++;
    int sx, sy; float z;
    GLOBE_PROJ(latS, lonS, 1.0f, sx, sy, z);
    if (segActive && z > 0 && pz > 0) {
      if ((unsigned)sx < (unsigned)screenW && (unsigned)sy < (unsigned)screenH &&
          (unsigned)psx < (unsigned)screenW && (unsigned)psy < (unsigned)screenH) {
        float zAvg = (z + pz) * 0.5f;
        uint16_t c = (zAvg > 0.50f) ? colBright : (zAvg > 0.18f) ? colMid : colDim;
        frameBuf.drawLine(psx, psy, sx, sy, c);
      }
    }
    psx = sx; psy = sy; pz = z;
    segActive = true;
  }
}

void drawScreen3DGlobe() {
  // Init sin table on first call
  if (!_gSinReady) {
    for (int i = 0; i <= 90; i++) _gSin[i] = sinf(i * DEG_TO_RAD);
    _gSinReady = true;
  }

  // --- Camera animation (time-based) ---
  static float camAngle = 0;
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  float dt = (lastMs == 0) ? 0.05f : (now - lastMs) / 1000.0f;
  if (dt > 0.5f) dt = 0.05f;
  lastMs = now;
  camAngle += 12.0f * dt;           // 12°/sec → 30s full orbit
  if (camAngle >= 360.0f) camAngle -= 360.0f;

  bool hasLoc = gps.location.isValid();
  float centerLon = hasLoc ? gps.location.lng() : 0;
  float centerLat = hasLoc ? gps.location.lat() : 20.0f;

  float camLonDeg = centerLon + camAngle;
  float camTiltDeg = 22.0f + centerLat * 0.35f;
  float camLonRad = camLonDeg * DEG_TO_RAD;
  float camTiltRad = camTiltDeg * DEG_TO_RAD;
  float sc = sinf(camLonRad), cc = cosf(camLonRad);
  float st = sinf(camTiltRad), ct = cosf(camTiltRad);

  const int gcx = screenW / 2;
  const int gcy = screenH / 2;
  const float ER = 32.0f;

  int camLon100 = (int)(camLonDeg * 100) % 36000;
  if (camLon100 < -18000) camLon100 += 36000;
  if (camLon100 >  18000) camLon100 -= 36000;

  // --- Ocean fill + atmosphere ---
  frameBuf.fillCircle(gcx, gcy, (int)ER, 0x0008);     // dark navy ocean
  frameBuf.drawCircle(gcx, gcy, (int)ER + 2, 0x0011);  // outer atmosphere
  frameBuf.drawCircle(gcx, gcy, (int)ER + 1, 0x001A);  // inner atmosphere

  // --- Latitude / longitude grid (30° spacing) ---
  {
    const uint16_t gridCol = 0x0926;  // very dim cyan
    // Latitude parallels
    for (int latD = -60; latD <= 60; latD += 30) {
      int latCD = latD * 100;
      int ppx = -1, ppy = -1; float ppz = -1; bool first = true;
      for (int lonD = -180; lonD <= 180; lonD += 12) {
        int sx, sy; float z;
        GLOBE_PROJ(latCD, lonD * 100, 1.0f, sx, sy, z);
        if (!first && z > 0 && ppz > 0)
          frameBuf.drawLine(ppx, ppy, sx, sy, gridCol);
        ppx = sx; ppy = sy; ppz = z; first = false;
      }
    }
    // Longitude meridians
    for (int lonD = 0; lonD < 360; lonD += 30) {
      int lonCD = (lonD <= 180 ? lonD : lonD - 360) * 100;
      int ppx = -1, ppy = -1; float ppz = -1; bool first = true;
      for (int latD = -90; latD <= 90; latD += 12) {
        int sx, sy; float z;
        GLOBE_PROJ(latD * 100, lonCD, 1.0f, sx, sy, z);
        if (!first && z > 0 && ppz > 0)
          frameBuf.drawLine(ppx, ppy, sx, sy, gridCol);
        ppx = sx; ppy = sy; ppz = z; first = false;
      }
    }
  }

  // --- Coastlines (bright green, limb-shaded) - simplified for globe ---
  globeRenderLayer(coastDataLow, sizeof(coastDataLow)/sizeof(int16_t),
      sc, cc, st, ct, gcx, gcy, ER, camLon100,
      0x07C0, 0x0380, 0x01C0);  // bright / mid / dim green

  // --- Country borders (dim yellow/amber) - simplified for globe ---
  globeRenderLayer(borderDataLow, sizeof(borderDataLow)/sizeof(int16_t),
      sc, cc, st, ct, gcx, gcy, ER, camLon100,
      0x4A49, 0x2924, 0x1482);  // bright / mid / dim amber

  // --- User position ---
  if (hasLoc) {
    int ux, uy; float uz;
    int uLatCD = (int)(gps.location.lat() * 100);
    int uLonCD = (int)(gps.location.lng() * 100);
    GLOBE_PROJ(uLatCD, uLonCD, 1.0f, ux, uy, uz);
    if (uz > 0) {
      frameBuf.fillCircle(ux, uy, 3, TFT_RED);
      frameBuf.drawCircle(ux, uy, 4, 0xF800);
    }
  }

  // --- Draw satellites at correct orbital positions ---
  // Orbital height ratios (Earth_R + alt) / Earth_R :
  //   GPS 4.17   GLONASS 4.00   Galileo 4.65   BeiDou 4.38
  // Visual: screenR = ER * (1 + (ratio-1)*0.22)
  if (hasLoc) {
    float uLatR = gps.location.lat() * DEG_TO_RAD;
    float uLonR = gps.location.lng() * DEG_TO_RAD;
    float sinUlat = sinf(uLatR), cosUlat = cosf(uLatR);

    for (size_t i = 0; i < satellites.size(); i++) {
      const SatData &sat = satellites[i];
      if (!sat.visible || sat.elevation < 1) continue;

      float orbitR; uint16_t col;
      if      (sat.system == "GPS")     { orbitR = 4.17f; col = TFT_YELLOW; }
      else if (sat.system == "GLONASS") { orbitR = 4.00f; col = TFT_CYAN;   }
      else if (sat.system == "Galileo") { orbitR = 4.65f; col = 0x54BF;     }
      else if (sat.system == "BeiDou")  { orbitR = 4.38f; col = TFT_ORANGE; }
      else                              { orbitR = 4.17f; col = TFT_WHITE;  }

      float visR = 1.0f + (orbitR - 1.0f) * 0.22f;
      float elR = sat.elevation * DEG_TO_RAD;
      float azR = sat.azimuth   * DEG_TO_RAD;
      float nadirAng = asinf(cosf(elR) / orbitR);
      float geocAng  = (float)M_PI * 0.5f - elR - nadirAng;
      float sinG = sinf(geocAng), cosG = cosf(geocAng);
      float sinAz = sinf(azR), cosAz = cosf(azR);
      float satLat = asinf(sinUlat * cosG + cosUlat * sinG * cosAz);
      float satLon = uLonR + atan2f(sinAz * sinG * cosUlat,
                                     cosG - sinUlat * sinf(satLat));

      // Project satellite and its ground point
      float cla = cosf(satLat), sla = sinf(satLat);
      float slo = sinf(satLon), clo = cosf(satLon);
      auto proj3 = [&](float r, int &ox, int &oy, float &oz) {
        float x = r*cla*slo, y = r*sla, zz = r*cla*clo;
        float x2 = x*cc + zz*sc, z2 = -x*sc + zz*cc;
        float y2 = y*ct - z2*st;
        oz = y*st + z2*ct;
        ox = gcx + (int)(x2 * ER);
        oy = gcy - (int)(y2 * ER);
      };

      int spx, spy, epx, epy; float spz, epz;
      proj3(visR, spx, spy, spz);
      proj3(1.0f, epx, epy, epz);

      // Tether line
      if (spz > 0 && epz > 0)
        frameBuf.drawLine(epx, epy, spx, spy, 0x2104);

      // Satellite dot (bigger for strong signals)
      if (spz > 0 && (unsigned)spx < (unsigned)screenW && (unsigned)spy < (unsigned)screenH) {
        int r = (sat.snr > 30) ? 3 : (sat.snr > 15) ? 2 : 1;
        frameBuf.fillCircle(spx, spy, r, col);
      }
    }
  }

  // --- Legend ---
  frameBuf.setTextSize(1);
  frameBuf.setTextColor(TFT_YELLOW); frameBuf.setCursor(2, screenH-28); frameBuf.print("GPS");
  frameBuf.setTextColor(TFT_CYAN);   frameBuf.setCursor(2, screenH-20); frameBuf.print("GLN");
  frameBuf.setTextColor(0x54BF);      frameBuf.setCursor(2, screenH-12); frameBuf.print("GAL");
  frameBuf.setTextColor(TFT_ORANGE); frameBuf.setCursor(26,screenH-28); frameBuf.print("BDS");

  // Sat count
  int visCnt = 0;
  for (size_t i = 0; i < satellites.size(); i++)
    if (satellites[i].visible) visCnt++;
  char buf[16];
  frameBuf.setTextColor(TFT_DARKGREY);
  snprintf(buf, sizeof(buf), "%d sats", visCnt);
  frameBuf.setCursor(screenW - (int)strlen(buf) * 6 - 2, 2);
  frameBuf.print(buf);
}

// ==================================================================
//  Screen dispatch and update
// ==================================================================

/*    Manage display elements drawing.
*/
void updateScreen(bool force = false) {
  static uint32_t lastDisplay = 0;
  // Faster updates for clock and NMEA monitor.
  unsigned long interval = 1000;
  if (currentScreen == SCR_GPS_CLOCK) interval = 200;
  else if (currentScreen == SCR_NMEA_MONITOR) interval = 200;
  else if (currentScreen == SCR_3D_GLOBE) interval = 50;

  if (force || millis() - lastDisplay > interval)
  {
    lastDisplay = millis();
    if (openMenu || configsMenu) return;
    // Clear screen for clean redraw.
    frameBuf.fillScreen(TFT_BLACK);
    // Dispatch to current screen.
    switch (currentScreen) {
      case SCR_MAIN:
        drawSatelliteDataTab();
        drawSkyPlot();
        break;
      case SCR_SKY_VIEW:     drawScreenSkyView();     break;
      case SCR_SIGNAL_BARS:  drawScreenSignalBars();  break;
      case SCR_FIX_SUMMARY:  drawScreenFixSummary();  break;
      case SCR_DASHBOARD:    drawScreenDashboard();   break;
      case SCR_COORDINATES:  drawScreenCoordinates(); break;
      case SCR_BREADCRUMB:   drawScreenBreadcrumb();  break;
      case SCR_ALT_PROFILE:  drawScreenAltProfile();  break;
      case SCR_SPEED_GRAPH:  drawScreenSpeedGraph();  break;
      case SCR_TRIP_STATS:   drawScreenTripStats();   break;
      case SCR_CONSTELLATION:drawScreenConstellation();break;
      case SCR_NMEA_MONITOR: drawScreenNmeaMonitor(); break;
      case SCR_GPS_CLOCK:    drawScreenGpsClock();    break;
      case SCR_GPS_MAP:      drawScreenGpsMap();      break;
      case SCR_3D_GLOBE:    drawScreen3DGlobe();    break;
      default: break;
    }
    frameBuf.pushSprite(0, 0);
  }
}

// ==================================================================
//  Header, status, menus
// ==================================================================

/*    Draw main features key map.
*/
void drawHeader(){
  int x = 1;
  int y = 1;
  int w = screenW;
  int h = 13;
  frameBuf.setTextSize(1);
  // Title.
  frameBuf.fillRect(x, y, w, h, TFT_GREEN); // bordo nel colore del testo
  frameBuf.setTextColor(TFT_BLACK);      // testo e sfondo
  frameBuf.setCursor(x + 4, y + 3);
  frameBuf.printf("%-1s", "    -= Cardputer ADV GPS Info =-");
  // Key map row.
  frameBuf.fillRect(x, y+h-1, w, h, TFT_BLACK);
  frameBuf.drawRect(x, y+h-1, w, h, TFT_GREEN);
  // Left side: key hints.
  frameBuf.setTextColor(TFT_GREEN);
  frameBuf.setCursor(x + 4, y+h + 3);
  frameBuf.print("[s]Start [c]Cfg [h]Help");
  // Right side: screen indicator.
  char scrInfo[20];
  snprintf(scrInfo, sizeof(scrInfo), "%d/%d %s%s",
    currentScreen + 1, SCR_COUNT, screenNames[currentScreen],
    autoSlideshow ? "*" : "");
  int scrInfoW = strlen(scrInfo) * 6;
  frameBuf.setTextColor(autoSlideshow ? TFT_YELLOW : TFT_GREEN);
  frameBuf.setCursor(screenW - scrInfoW - 4, y+h + 3);
  frameBuf.print(scrInfo);
}

/*    Draw app features status.
*/
void drawStatus(){
  int x = 1;
  int y = 122;
  int w = screenW;
  int h = 13;
  char statusChar[64];
  statusChar[0] = '\0';
  const char* gpsStr = "Off";
  if (gpsSerialState == GPS_ON) {
    if (gsaFixMode == 3) gpsStr = "3D";
    else if (gsaFixMode == 2) gpsStr = "2D";
    else gpsStr = "On";
  }
  else if (gpsSerialState == GPS_ERR) gpsStr = "Err";
  snprintf(statusChar + strlen(statusChar), sizeof(statusChar) - strlen(statusChar),"GP:%s ", gpsStr);
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar),"Rx:%d Tx:%d ", gpsRxPin, gpsTxPin);
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar),"Bd:%dk", gpsBaud / 1000);
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar)," | Cnsl:%s", debugSerial ? "On" : "Off");
  frameBuf.fillRect(x, y, w, h, TFT_DARKGREY);
  frameBuf.setTextColor(TFT_WHITE);
  frameBuf.setCursor(x + 4, y + 3);
  frameBuf.setTextSize(1);
  frameBuf.printf("%-1s", statusChar);
}

/*    Draw configuration popup.
*/
void drawConfig(bool should_I) {
  if (should_I == true) {
    if (gpsSerial) { // If active, stop it.
      gpsSerial = false;
      initGPSSerial(false);
    }
    frameBuf.fillRect(10, 10, screenW-20, screenH-20, TFT_BLACK);
    frameBuf.drawRect(12, 12, screenW-24, screenH-24, TFT_GREEN);
    frameBuf.setTextColor(TFT_WHITE, TFT_BLACK);
    frameBuf.setTextSize(1);
    frameBuf.setCursor(25, 25);
    frameBuf.println("Configurations:");
    frameBuf.setCursor(25, 35);
    frameBuf.println("Nav: [Up/Dow]. Val: [0-9].");
    frameBuf.setCursor(25, 45);
    frameBuf.println("Exit: [c]. Save: [ok].");
    frameBuf.setCursor(25, 70);
    frameBuf.printf("ADV RX pin (act:%d): %s %s", gpsRxPin, configsTmp[0].c_str(), configsMenuSel == 0 ? "<" : " ");
    frameBuf.setCursor(25, 80);
    frameBuf.printf("ADV TX pin (act:%d): %s %s", gpsTxPin, configsTmp[1].c_str(), configsMenuSel == 1 ? "<" : " ");
    frameBuf.setCursor(25, 90);
    frameBuf.printf("ADV Baud (act:%d): %s %s", gpsBaud, configsTmp[2].c_str(), configsMenuSel == 2 ? "<" : " ");
  }
  else
    updateScreen(true); // Forced update.
}

/*    Save screenshot as 24-bit BMP to SD card.
      Returns true on success.
*/
void sendScreenshotSerial() {
  const int W = 240, H = 135;
  Serial.flush();
  delay(50);
  Serial.println("SCREENSHOT_START");
  Serial.printf("%d %d\n", W, H);
  Serial.flush();
  char hexRow[W * 4 + 1];
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      uint16_t c = (uint16_t)frameBuf.readPixel(x, y);
      sprintf(&hexRow[x * 4], "%04X", c);
    }
    Serial.println(hexRow);
    if (y % 10 == 9) { Serial.flush(); delay(5); }
  }
  Serial.flush();
  delay(50);
  Serial.println("SCREENSHOT_END");
  Serial.flush();
}


/*    Draw info popup.
*/
void drawInfo(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    const char* helpText[] = {
      "Cardputer ADV GPS Info",
      "V " APP_VERSION " (ADV) by alcor55",
      "",
      "Github:",
      "https://github.com/alcor55",
      "/Cardputer-GPS-Info"
    };
    int count = sizeof(helpText) / sizeof(helpText[0]);
    frameBuf.fillRect(18, 18, 204, 99, TFT_BLACK);
    frameBuf.drawRect(20, 20, 200, 95, TFT_GREEN);
    frameBuf.setTextColor(TFT_WHITE, TFT_BLACK);
    frameBuf.setTextSize(1);
    frameBuf.setCursor(25, 24);
    int y = 24;
    for (int i = 0; i < count; i++) {
      frameBuf.setCursor(25, y);
      frameBuf.println(helpText[i]);
      y += 10;
    }
  } else {
    openMenu = false;
    updateScreen(true); // Forced update.
  }
}

/*    Draw help popup.
*/
void drawHelp(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    const char* helpText[] = {
      "[s] Start/Stop the GPS (serial).",
      "[c] Configuration menu.",
      "[h] Help menu (this).",
      "[i] Info menu.",
      "[<] Previous screen.",
      "[>] Next screen.",
      "[.] Toggle auto-slideshow (5s).",
      "[0-9] Jump to screen 0-9.",
      "[p] Show/hide ID on skyplot.",
      "[o] Show/hide System on skyplot.",
      "[l] USB serial satellites list.",
      "[n] USB serial NMEA sentences."
    };
    int count = sizeof(helpText) / sizeof(helpText[0]);
    frameBuf.fillRect(4, 4, screenW-8, screenH-8, TFT_BLACK);
    frameBuf.drawRect(6, 6, screenW-12, screenH-12, TFT_GREEN);
    frameBuf.setTextColor(TFT_WHITE, TFT_BLACK);
    frameBuf.setTextSize(1);
    int y = 12;
    for (int i = 0; i < count; i++) {
      String line = helpText[i];
      int endKey = line.indexOf(']') + 1;
      if (endKey > 0) {
        String key = line.substring(0, endKey);
        String desc = line.substring(endKey);
        frameBuf.setTextColor(TFT_GREEN, TFT_BLACK);
        frameBuf.setCursor(14, y);
        frameBuf.print(key);
        frameBuf.setTextColor(TFT_WHITE, TFT_BLACK);
        frameBuf.print(desc);
      } else {
        frameBuf.setTextColor(TFT_WHITE, TFT_BLACK);
        frameBuf.setCursor(20, y);
        frameBuf.print(line);
      }
      y += 10;
    }
  } else {
    openMenu = false;
    updateScreen(true); // Forced update.
  }
}

// ==================================================================
//  Key handling
// ==================================================================

/*    Handle keyboard data inputs.
*/
void handleKeys() {
  if (configsMenu) {
    if (M5Cardputer.Keyboard.isChange()) {
      if (M5Cardputer.Keyboard.isPressed()) {
        Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();
        for (auto c : status.word) {
          // Arrow selection vertical up.
          if (c == ';' || c == '.')
            configsMenuSel = (configsMenuSel + 1) % 3;
          // Numbers 0-9.
          if (c >= 48 && c <= 57) {
            configsTmp[configsMenuSel] += c;
          }
        }
        // Delete.
        if (status.del && configsTmp[configsMenuSel].length() > 0)
          configsTmp[configsMenuSel].remove(configsTmp[configsMenuSel].length() - 1);
        // Store.
        if (status.enter) {
          if (configsTmp[0].length() > 0)
            gpsRxPin = configsTmp[0].toInt();
          if (configsTmp[1].length() > 0)
            gpsTxPin = configsTmp[1].toInt();
          if (configsTmp[2].length() > 0)
            gpsBaud = configsTmp[2].toInt();
          configsTmp[0] = configsTmp[1] = configsTmp[2] = "";
          configsMenu = false;
          updateScreen(true);
          return;
        }
        drawConfig(true);
        frameBuf.pushSprite(0, 0);
      }
    }
  }
}

/*    Handle app functions.
*/
void handleControls() {
  // Handle S key for Start/Stop.
  if (M5Cardputer.Keyboard.isKeyPressed('s')) {
    gpsSerial = !gpsSerial;
    initGPSSerial(gpsSerial);
    gpsSerialState = gpsSerial ? GPS_ON : GPS_OFF;
    frameBuf.pushSprite(0, 0);
    delay(300);
  }
  // Handle C key for Config.
  if (M5Cardputer.Keyboard.isKeyPressed('c')) {
    configsMenu = !configsMenu; // Invert status.
    drawConfig(configsMenu);
    frameBuf.pushSprite(0, 0);
    delay(300); // Debounce.
  }
  // Handle H key for Help.
  if (M5Cardputer.Keyboard.isKeyPressed('h')) {
    helpMenu = !helpMenu; // Invert status.
    drawHelp(helpMenu);
    frameBuf.pushSprite(0, 0);
    delay(300); // Debounce.
  }
  // Handle I key for Info.
  if (M5Cardputer.Keyboard.isKeyPressed('i')) {
    infoMenu = !infoMenu; // Invert status.
    drawInfo(infoMenu);
    frameBuf.pushSprite(0, 0);
    delay(300); // Debounce.
  }
  // Handle P key to hide id from plot.
  if (M5Cardputer.Keyboard.isKeyPressed('p')) {
    hidePlotId = !hidePlotId; // Invert status.
    delay(300); // Debounce.
  }
  // Handle O key to hide system from plot.
  if (M5Cardputer.Keyboard.isKeyPressed('o')) {
    hidePlotSystem = !hidePlotSystem; // Invert status.
    delay(300); // Debounce.
  }
  // Handle L key for Serial satellite list.
  if (M5Cardputer.Keyboard.isKeyPressed('l')) {
    nmeaSerial = false;
    satListSerial = !satListSerial; // Invert status.
    initDebugSerial(satListSerial);
    frameBuf.pushSprite(0, 0);
    delay(300); // Debounce.
  }
  // Handle N key for serial NMEA sentences dump.
  if (M5Cardputer.Keyboard.isKeyPressed('n')) {
    satListSerial = false;
    nmeaSerial = !nmeaSerial; // Invert status.
    initDebugSerial(nmeaSerial);
    frameBuf.pushSprite(0, 0);
    delay(300); // Debounce.
  }
  // ---- Screen navigation (only when no menu is open) ----
  if (!openMenu && !configsMenu) {
    // Previous screen (left arrow = ',').
    if (M5Cardputer.Keyboard.isKeyPressed(',')) {
      currentScreen = (ScreenID)((currentScreen - 1 + SCR_COUNT) % SCR_COUNT);
      lastSlideChange = millis();
      updateScreen(true);
      delay(300);
    }
    // Next screen (right arrow = '/').
    if (M5Cardputer.Keyboard.isKeyPressed('/')) {
      currentScreen = (ScreenID)((currentScreen + 1) % SCR_COUNT);
      lastSlideChange = millis();
      updateScreen(true);
      delay(300);
    }
    // Toggle auto-slideshow ('.').
    if (M5Cardputer.Keyboard.isKeyPressed('.')) {
      autoSlideshow = !autoSlideshow;
      lastSlideChange = millis();
      frameBuf.pushSprite(0, 0);
      delay(300);
    }
    // Direct screen jump with number keys 0-9.
    for (char digit = '0'; digit <= '9'; digit++) {
      if (M5Cardputer.Keyboard.isKeyPressed(digit)) {
        int scrNum = digit - '0';
        if (scrNum < SCR_COUNT) {
          currentScreen = (ScreenID)scrNum;
          lastSlideChange = millis();
          updateScreen(true);
          delay(300);
        }
        break;
      }
    }
    // Map zoom controls (z = zoom in, x = zoom out).
    if (currentScreen == SCR_GPS_MAP) {
      if (M5Cardputer.Keyboard.isKeyPressed('z')) {
        if (mapZoom < 5) mapZoom++;
        updateScreen(true);
        delay(300);
      }
      if (M5Cardputer.Keyboard.isKeyPressed('x')) {
        if (mapZoom > 0) mapZoom--;
        updateScreen(true);
        delay(300);
      }
    }
    // Enter key: send screenshot over serial.
    if (M5Cardputer.Keyboard.isKeyPressed(KEY_ENTER)) {
      sendScreenshotSerial();
      frameBuf.fillRect(70, 55, 100, 25, TFT_BLACK);
      frameBuf.drawRect(70, 55, 100, 25, TFT_GREEN);
      frameBuf.setTextColor(TFT_GREEN);
      frameBuf.setCursor(82, 62);
      frameBuf.print("Sent!");
      frameBuf.pushSprite(0, 0);
      delay(1000);
      updateScreen(true);
    }
  }
}

// ==================================================================
//  Setup and loop
// ==================================================================

/*    Standard setup function.
*/
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg); // Init Cardputer ADV.
  Serial.begin(115200);   // Init USB CDC for screenshot transfer.
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setBrightness(128); // Set brightness 0-255.
  frameBuf.createSprite(240, 135);
  lastValidGpsMillis = millis();
  lastSlideChange = millis();
  // Init NMEA buffer.
  memset(nmeaBuf, 0, sizeof(nmeaBuf));
  // Init IMU (BMI270).
  if (M5.Imu.isEnabled()) {
    imuAvailable = true;
    M5.Imu.loadOffsetFromNVS();
  }
  // Auto-start GPS serial.
  gpsSerial = true;
  initGPSSerial(true);
  gpsSerialState = GPS_ON;
  updateScreen(true); // Forced first update.
}

/*    Standard loop.
*/
void loop() {
  M5Cardputer.update(); // Update Cardputer state (keyboard, buttons, sensors etc...).
  // Read IMU.
  if (imuAvailable) {
    M5.Imu.getAccel(&imuAx, &imuAy, &imuAz);
    M5.Imu.getGyro(&imuGx, &imuGy, &imuGz);
    M5.Imu.getTemp(&imuTemp);
    imuPitch = atan2(-imuAx, sqrt(imuAy * imuAy + imuAz * imuAz)) * 180.0 / M_PI;
    imuRoll = atan2(imuAy, imuAz) * 180.0 / M_PI;
    imuGforce = sqrt(imuAx * imuAx + imuAy * imuAy + imuAz * imuAz);
    if (imuGforce > tripMaxG) tripMaxG = imuGforce;
  }
  if (gpsSerial){
    serialGPSRead();
    recordTrackPoint();
    updateTripStats();
    serialConsoleSatsList();
  }
  updateScreen();
  // Auto-slideshow advance.
  if (autoSlideshow && !openMenu && !configsMenu &&
      millis() - lastSlideChange >= slideshowInterval) {
    lastSlideChange = millis();
    currentScreen = (ScreenID)((currentScreen + 1) % SCR_COUNT);
    updateScreen(true);
  }
  handleControls();
  handleKeys();
  delay(10); // Small delay to avoid busy loop.
}

/*
  That's all folks!
*/
