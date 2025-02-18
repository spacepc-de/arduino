#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024

#include <TimeLib.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <math.h>

#define SerialMon Serial
#define SerialAT  Serial1

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024
#define GSM_PIN ""
const char apn[] = "APN";
const char gprsUser[] = "";
const char gprsPass[] = "";
const char* broker = "mqtt.eu.thingsboard.cloud";
const char* topicTelemetry = "v1/devices/me/telemetry";
const char* clientID = "CLient_ID";

#define UART_BAUD 9600
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4
#define LED_PIN 12
#define BAT_ADC_PIN 35

// Sendeintervall in Millisekunden
const unsigned long SEND_INTERVAL_MS = 300000;
#define DBG(x) SerialMon.println(x)

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);

time_t parseTimestamp(String tStr) {
  if (tStr.length() < 14) return 0;
  int year = tStr.substring(0,4).toInt(),
      month = tStr.substring(4,6).toInt(),
      day = tStr.substring(6,8).toInt(),
      hour = tStr.substring(8,10).toInt(),
      minute = tStr.substring(10,12).toInt(),
      second = tStr.substring(12,14).toInt();
  tmElements_t tm;
  tm.Second = second; tm.Minute = minute; tm.Hour = hour; tm.Day = day; tm.Month = month; tm.Year = year - 1970;
  return makeTime(tm);
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1), dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(lat1))*cos(radians(lat2))*
             sin(dLon/2)*sin(dLon/2);
  return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

void modemPowerOn() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(PWR_PIN, LOW);
}
void modemPowerOff() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(1500);
  digitalWrite(PWR_PIN, LOW);
}
void modemRestart() { modemPowerOff(); delay(1000); modemPowerOn(); }

void enableGPS() {
  modem.sendAT("+CGPIO=0,48,1,1");
  if (modem.waitResponse(10000L) != 1) DBG("Set GPS Power HIGH Failed");
  modem.enableGPS();
}
void disableGPS() {
  modem.sendAT("+CGPIO=0,48,1,0");
  if (modem.waitResponse(10000L) != 1) DBG("Set GPS Power LOW Failed");
  modem.disableGPS();
}

void setup() {
  SerialMon.begin(115200); delay(10);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
  pinMode(BAT_ADC_PIN, INPUT);
  modemPowerOn();
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  SerialMon.println("Initialisiere Modem und Netzwerk...");
  delay(10000);
  if (!modem.testAT()) { SerialMon.println("Modem reagiert nicht, starte neu..."); modemRestart(); }
  SerialMon.println("Warte auf Netzwerk...");
  if (!modem.waitForNetwork(60000L)) SerialMon.println("Kein Netzwerk verfügbar.");
  else SerialMon.println("Netzwerk gefunden.");
  SerialMon.println("Setze NB-IoT Modus...");
  modem.sendAT("+CMNB=2");
  if (modem.waitResponse(2000L) != 1) SerialMon.println("NB-IoT Modus konnte nicht gesetzt werden.");
  else SerialMon.println("NB-IoT Modus gesetzt.");
  SerialMon.print("Verbinde zu APN: "); SerialMon.println(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) SerialMon.println("NB-IoT Verbindung fehlgeschlagen.");
  else SerialMon.println("NB-IoT verbunden.");
  mqtt.setServer(broker, 1883);
}

void loop() {
  if (!modem.testAT()) { SerialMon.println("Modem nicht erreichbar, starte neu..."); modemRestart(); return; }
  if (!mqtt.connected()) {
    SerialMon.println("MQTT nicht verbunden, versuche Verbindung aufzubauen...");
    if (mqtt.connect(clientID)) SerialMon.println("MQTT verbunden.");
    else { SerialMon.println("MQTT Verbindung fehlgeschlagen."); delay(5000); return; }
  }
  mqtt.loop();
  SerialMon.println("Starte GPS-Positionssuche. Stelle sicher, dass du im Freien bist.");
  enableGPS();
  float lat = 0.0, lon = 0.0;
  while (true) {
    if (modem.getGPS(&lat, &lon)) { SerialMon.println("GPS-Fix erreicht."); break; }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(2000);
    SerialMon.println("GPS-Fix wird gesucht...");
  }
  
  String response = "";
  modem.sendAT("+CGNSINF");
  unsigned long timeout = millis() + 3000;
  while (millis() < timeout) {
    while (SerialAT.available()) response += (char)SerialAT.read();
    if (response.indexOf("OK") != -1 || response.indexOf("+CGNSINF:") != -1) break;
  }
  SerialMon.println("Empfangene GPS-Daten: " + response);
  
  int idx = response.indexOf("+CGNSINF:");
  String utcTime = "";
  if (idx >= 0) {
    String gpsLine = response.substring(idx);
    int endLine = gpsLine.indexOf("\n");
    if (endLine > 0) gpsLine = gpsLine.substring(0, endLine);
    int colonIndex = gpsLine.indexOf(":");
    if (colonIndex != -1) gpsLine = gpsLine.substring(colonIndex + 1);
    gpsLine.trim();
    const int maxTokens = 20;
    String tokens[maxTokens];
    int tokenCount = 0, startIndex = 0;
    while (tokenCount < maxTokens) {
      int commaIndex = gpsLine.indexOf(',', startIndex);
      if (commaIndex == -1) { tokens[tokenCount++] = gpsLine.substring(startIndex); break; }
      tokens[tokenCount++] = gpsLine.substring(startIndex, commaIndex);
      startIndex = commaIndex + 1;
    }
    if (tokenCount > 2) utcTime = tokens[2];
  }
  
  if (utcTime.length() == 0) {
    String timeResponse = "";
    modem.sendAT("AT+CCLK?");
    unsigned long timeTimeout = millis() + 3000;
    while (millis() < timeTimeout) {
      while (SerialAT.available()) timeResponse += (char)SerialAT.read();
      if (timeResponse.indexOf("OK") != -1 || timeResponse.indexOf("+CCLK:") != -1) break;
    }
    SerialMon.println("Empfangene GSM-Zeit: " + timeResponse);
    const int maxTokens = 10;
    String tokens[maxTokens];
    int tokenCount = 0, startIndex = 0;
    timeResponse.trim();
    while (tokenCount < maxTokens) {
      int commaIndex = timeResponse.indexOf(',', startIndex);
      if (commaIndex == -1) { tokens[tokenCount++] = timeResponse.substring(startIndex); break; }
      tokens[tokenCount++] = timeResponse.substring(startIndex, commaIndex);
      startIndex = commaIndex + 1;
    }
    if (tokenCount >= 3) {
      utcTime = tokens[2];
      int dotIdx = utcTime.indexOf('.');
      if (dotIdx != -1) utcTime = utcTime.substring(0, dotIdx);
    }
  }
  
  time_t unixTime = parseTimestamp(utcTime);
  String unixTimeStr = String(unixTime);
  int rawBattery = analogRead(BAT_ADC_PIN);
  float batteryVoltage = rawBattery * (3.3 / 4095.0) * 2;
  String batteryStr = String(batteryVoltage, 2);
  String payload = "{\"utc\":" + unixTimeStr + ",\"lat\":" + String(lat,6) +
                   ",\"lon\":" + String(lon,6) + ",\"bat\":" + batteryStr + "}";
  
  static bool firstReading = true;
  static double lastLat = 0.0, lastLon = 0.0;
  double distance = firstReading ? 0.0 : haversineDistance(lastLat, lastLon, lat, lon);
  if (firstReading || distance >= 50) {
    if (!firstReading) { SerialMon.print("Positionsänderung: "); SerialMon.print(distance); SerialMon.println(" Meter."); }
    mqtt.publish(topicTelemetry, payload.c_str());
    SerialMon.println("Telemetry veröffentlicht: " + payload);
    lastLat = lat; lastLon = lon; firstReading = false;
  }
  else {
    SerialMon.print("Positionsänderung zu gering ("); SerialMon.print(distance); SerialMon.println(" Meter). Keine Übertragung.");
  }
  disableGPS();
  delay(SEND_INTERVAL_MS);
}
