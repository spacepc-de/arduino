/**************************************************************
 *
 * Beispielcode für Lilygo T-SIM (SIM7000):
 * Es werden die GPS-Koordinaten (UTC, LAT, LON) sowie der
 * Batteriestand via MQTT an das Topic "GsmClientTest/coords"
 * gesendet – jedoch nur, wenn sich die Position um mindestens
 * 50 Meter geändert hat.
 *
 * NB‑IoT wird verwendet, indem der NB‑IoT‑Modus (AT+CMNB=2)
 * gesetzt wird, bevor modem.gprsConnect() aufgerufen wird.
 *
 **************************************************************/

// Serielle Schnittstellen: Debug über Serial, AT-Befehle über Serial1
#define SerialMon Serial
#define SerialAT  Serial1

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024

// SIM-PIN, falls benötigt
#define GSM_PIN ""

// NB‑IoT-Zugangsdaten (bitte anpassen)
const char apn[]      = "YOUR-APN";
const char gprsUser[] = "";
const char gprsPass[] = "";

// MQTT-Details
const char* broker      = "broker.hivemq.com";
const char* topicCoords = "GsmClientTest/coords";

// Hardware-Definitionen
#define UART_BAUD 9600
#define PIN_TX    27
#define PIN_RX    26
#define PWR_PIN   4
#define LED_PIN   12

// Debug-Ausgabe-Makro
#define DBG(x) SerialMon.println(x)

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>
#include <math.h>   // Für mathematische Funktionen

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Berechnet die Distanz zwischen zwei GPS-Koordinaten (in Metern)
double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Erdradius in Metern
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// --- Funktionen zum Ein- und Ausschalten des Modems/GPS ---

void modemPowerOn() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(1000);  // Ton ca. 1 Sekunde
  digitalWrite(PWR_PIN, LOW);
}

void modemPowerOff() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(1500);  // ca. 1,5 Sekunden
  digitalWrite(PWR_PIN, LOW);
}

void modemRestart() {
  modemPowerOff();
  delay(1000);
  modemPowerOn();
}

void enableGPS() {
  // Schaltet die GPS-Stromversorgung ein
  modem.sendAT("+CGPIO=0,48,1,1");
  if (modem.waitResponse(10000L) != 1) {
    DBG("Set GPS Power HIGH Failed");
  }
  modem.enableGPS();
}

void disableGPS() {
  // Schaltet die GPS-Stromversorgung aus
  modem.sendAT("+CGPIO=0,48,1,0");
  if (modem.waitResponse(10000L) != 1) {
    DBG("Set GPS Power LOW Failed");
  }
  modem.disableGPS();
}


// --- Setup: Modem, NB‑IoT und MQTT konfigurieren ---

void setup() {
  SerialMon.begin(115200);
  delay(10);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // Modem einschalten
  modemPowerOn();
  
  // Seriellen Port für AT-Befehle starten
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  SerialMon.println("Initialisiere Modem und Netzwerk...");
  delay(10000);  // Wartezeit für Stabilisierung

  // Teste AT-Befehle
  if (!modem.testAT()) {
    SerialMon.println("Modem reagiert nicht, starte neu...");
    modemRestart();
  }

  // Warten auf Netzregistrierung
  SerialMon.println("Warte auf Netzwerk...");
  if (!modem.waitForNetwork(60000L)) {
    SerialMon.println("Kein Netzwerk verfügbar.");
  } else {
    SerialMon.println("Netzwerk gefunden.");
  }

  // NB‑IoT-Modus aktivieren (SIM7000: AT+CMNB=2 für NB‑IoT)
  SerialMon.println("Setze NB-IoT Modus...");
  modem.sendAT("+CMNB=2");
  if (modem.waitResponse(2000L) != 1) {
    SerialMon.println("NB-IoT Modus konnte nicht gesetzt werden.");
  } else {
    SerialMon.println("NB-IoT Modus gesetzt.");
  }

  // NB‑IoT-Verbindung herstellen (mittels gprsConnect, da die Library keinen separaten nbiotConnect-Aufruf hat)
  SerialMon.print("Verbinde zu APN: ");
  SerialMon.println(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("NB-IoT Verbindung fehlgeschlagen.");
  } else {
    SerialMon.println("NB-IoT verbunden.");
  }

  // MQTT-Broker konfigurieren
  mqtt.setServer(broker, 1883);
}


// --- Loop: GPS-Fix ermitteln, UTC-Zeit, Batteriestand abrufen
//         und Daten nur bei einer Änderung >=50m senden ---

void loop() {
  // Prüfe, ob das Modem reagiert
  if (!modem.testAT()) {
    SerialMon.println("Modem nicht erreichbar, starte neu...");
    modemRestart();
    return;
  }

  // Sicherstellen, dass eine MQTT-Verbindung besteht
  if (!mqtt.connected()) {
    SerialMon.println("MQTT nicht verbunden, versuche Verbindung aufzubauen...");
    if (mqtt.connect("GsmClientSim")) {
      SerialMon.println("MQTT verbunden.");
    } else {
      SerialMon.println("MQTT Verbindung fehlgeschlagen.");
      delay(5000);
      return;
    }
  }
  mqtt.loop();

  SerialMon.println("Starte GPS-Positionssuche. Stelle sicher, dass du im Freien bist.");
  enableGPS();

  float lat = 0.0, lon = 0.0;
  // Warte, bis ein GPS-Fix erreicht wurde
  while (1) {
    if (modem.getGPS(&lat, &lon)) {
      SerialMon.println("GPS-Fix erreicht.");
      break;
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // LED blinkt während der Suche
    delay(2000);
    SerialMon.println("GPS-Fix wird gesucht...");
  }

  // Hole erweiterte GPS-Daten, um die UTC-Zeit zu erhalten
  String response = "";
  modem.sendAT("+CGNSINF");
  unsigned long timeout = millis() + 3000;
  while (millis() < timeout) {
    while (SerialAT.available()) {
      response += (char)SerialAT.read();
    }
    if (response.indexOf("OK") != -1 || response.indexOf("+CGNSINF:") != -1) {
      break;
    }
  }
  SerialMon.println("Empfangene GPS-Daten: " + response);

  int idx = response.indexOf("+CGNSINF:");
  String utcTime = "";
  if (idx >= 0) {
    // Extrahiere die Zeile mit den GPS-Daten
    String gpsLine = response.substring(idx);
    int endLine = gpsLine.indexOf("\n");
    if (endLine > 0) {
      gpsLine = gpsLine.substring(0, endLine);
    }
    int colonIndex = gpsLine.indexOf(":");
    if (colonIndex != -1) {
      gpsLine = gpsLine.substring(colonIndex + 1);
    }
    gpsLine.trim();

    // Zerlege die Zeile anhand der Kommata
    const int maxTokens = 20;
    String tokens[maxTokens];
    int tokenCount = 0;
    int startIndex = 0;
    while (tokenCount < maxTokens) {
      int commaIndex = gpsLine.indexOf(',', startIndex);
      if (commaIndex == -1) {
        tokens[tokenCount++] = gpsLine.substring(startIndex);
        break;
      }
      tokens[tokenCount++] = gpsLine.substring(startIndex, commaIndex);
      startIndex = commaIndex + 1;
    }
    // tokens[2] sollte die UTC-Zeit enthalten (wenn vorhanden)
    if (tokenCount > 2) {
      utcTime = tokens[2];
    }
  }

  // --- Batteriestand abfragen via AT+Befehl +CBC ---
  String battResponse = "";
  modem.sendAT("+CBC");
  unsigned long battTimeout = millis() + 3000;
  while (millis() < battTimeout) {
    while (SerialAT.available()) {
      battResponse += (char)SerialAT.read();
    }
    if (battResponse.indexOf("OK") != -1 || battResponse.indexOf("+CBC:") != -1) {
      break;
    }
  }
  SerialMon.println("Empfangene Batterie-Daten: " + battResponse);
  String batteryLevel = "";
  int idxBatt = battResponse.indexOf("+CBC:");
  if (idxBatt >= 0) {
    String battLine = battResponse.substring(idxBatt);
    int endLineBatt = battLine.indexOf("\n");
    if (endLineBatt > 0) {
      battLine = battLine.substring(0, endLineBatt);
    }
    int comma1 = battLine.indexOf(',');
    int comma2 = battLine.indexOf(',', comma1 + 1);
    if (comma1 != -1 && comma2 != -1) {
      batteryLevel = battLine.substring(comma1 + 1, comma2);
      batteryLevel.trim();
    }
  }
  
  // Erstelle die Nachricht (ohne FIX/SAT) inklusive Batteriestand
  String message = "UTC:" + utcTime +
                   ",LAT:" + String(lat, 6) +
                   ",LON:" + String(lon, 6) +
                   ",BAT:" + batteryLevel;

  // Übertrage die GPS-Daten nur, wenn sich die Position um mindestens 50 Meter geändert hat
  static bool firstReading = true;
  static double lastLat = 0.0, lastLon = 0.0;
  double distance = 0.0;
  if (!firstReading) {
    distance = haversineDistance(lastLat, lastLon, lat, lon);
  }
  if (firstReading || distance >= 50) {
    if (!firstReading) {
      SerialMon.print("Positionsänderung: ");
      SerialMon.print(distance);
      SerialMon.println(" Meter.");
    }
    mqtt.publish(topicCoords, message.c_str());
    SerialMon.println("GPS-Daten veröffentlicht: " + message);
    lastLat = lat;
    lastLon = lon;
    firstReading = false;
  } else {
    SerialMon.print("Positionsänderung zu gering (");
    SerialMon.print(distance);
    SerialMon.println(" Meter). Keine Übertragung.");
  }

  disableGPS();

  // Warte 60 Sekunden, bevor der nächste Messvorgang gestartet wird
  delay(60000);
}
