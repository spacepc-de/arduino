#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>

const int moistureSensorPin = 35;

Preferences preferences;
int airValue;
int waterValue;
unsigned long startTime;
const unsigned long calibrationPeriod = 20000;
const unsigned long sleepTimeSeconds = 300;

const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* mqttServer = "MQTTServerAddress";
const char* mqttUser = "MQTTUsername";
const char* mqttPassword = "MQTTPassword";
const char* mqttTopic = "MQTTTopic";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  preferences.begin("moisture", false);

  airValue = preferences.getInt("airValue", 4095);
  waterValue = preferences.getInt("waterValue", 0);
  bool isCalibrated = preferences.getBool("isCalibrated", false);

  if (!isCalibrated) {
    Serial.println("Calibration started - Please put the sensor in different moisture states.");
    startTime = millis();
  } else {
    Serial.println("Calibration skipped.");
  }

  setup_wifi();
  client.setServer(mqttServer, 1883);
}

void loop() {
  bool isCalibrated = preferences.getBool("isCalibrated", false);

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (!isCalibrated && millis() - startTime < calibrationPeriod) {
    int sensorValue = analogRead(moistureSensorPin);
    waterValue = max(waterValue, sensorValue);
    airValue = min(airValue, sensorValue);
    Serial.println("Calibration: Air Value = " + String(airValue) + ", Water Value = " + String(waterValue));
  } else if (millis() - startTime >= calibrationPeriod && !isCalibrated) {
    preferences.putInt("airValue", airValue);
    preferences.putInt("waterValue", waterValue);
    preferences.putBool("isCalibrated", true);
  }

  if (isCalibrated) {
    int sensorValue = analogRead(moistureSensorPin);
    int moisturePercent = waterValue == airValue ? 0 : map(sensorValue, airValue, waterValue, 100, 0);
    moisturePercent = constrain(moisturePercent, 0, 100);
    Serial.println("Soil Moisture: " + String(moisturePercent) + "%");

    char msg[50];
    sprintf(msg, "%d", moisturePercent);
    client.publish(mqttTopic, msg);

    client.loop();
    delay(10);

    esp_sleep_enable_timer_wakeup(sleepTimeSeconds * 1000000);
    esp_deep_sleep_start();
  }
}

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected.");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting to connect to MQTT...");
    if (client.connect("ArduinoClient", mqttUser, mqttPassword)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds");
      delay(5000);
    }
  }
}
