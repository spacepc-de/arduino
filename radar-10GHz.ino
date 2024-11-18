const int sensorPin = 16;  // GPIO-Pin, an dem der Sensor angeschlossen ist
const float frequency = 10.525e9;  // Frequenz des Mikrowellen-Sensors in Hz
const float c = 3e8;  // Lichtgeschwindigkeit in m/s
const int measureDuration = 100;  // Messdauer in Millisekunden
const int numMeasurements = 10;  // Anzahl der Messungen zur Mittelung

volatile unsigned long pulseCount = 0;

void IRAM_ATTR handlePulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);  // Serielle Kommunikation mit 115200 Baudrate
  pinMode(sensorPin, INPUT);  // Sensor-Pin als Eingang festlegen
  attachInterrupt(digitalPinToInterrupt(sensorPin), handlePulse, RISING);
}

void loop() {
  float totalSpeed = 0.0;
  unsigned long startTime = millis();  // Startzeit für die Schleife

  for (int i = 0; i < numMeasurements; i++) {
    unsigned long measurementStartTime = millis();  // Startzeit für die Messung
    pulseCount = 0;

    // 100 ms messen
    delay(measureDuration);

    unsigned long duration = millis() - measurementStartTime;
    unsigned long pulses = pulseCount;

    float pulseFrequency = (float)pulses / duration * 1000;  // Pulsfrequenz in Hz
    float speed_m_s = pulseFrequency * c / (2 * frequency);  // Geschwindigkeit in m/s
    float speed_km_h = speed_m_s * 3.6;  // Geschwindigkeit in km/h

    totalSpeed += speed_km_h;
  }

  float averageSpeed = totalSpeed / numMeasurements;

  Serial.print("Average Speed: ");
  Serial.print(averageSpeed);
  Serial.println(" km/h");

  // Warten Sie für den Rest der Sekunde, damit die Ausgabe jede Sekunde erfolgt
  unsigned long endTime = millis();
  unsigned long loopDuration = endTime - startTime;
  if (loopDuration < 1000) {
    delay(1000 - loopDuration);
  }
}
