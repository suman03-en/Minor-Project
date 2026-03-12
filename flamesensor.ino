/*************************************************
 * ESP32-S3 Flame Sensor Interface
 * AO  -> GPIO 1 (ADC1_CH0)
 * DO  -> GPIO 3
 *************************************************/

#define FLAME_DO  3    // Digital output

void setup() {
  Serial.begin(115200);

  // Configure digital pin
  pinMode(FLAME_DO, INPUT);

  Serial.println("Flame Sensor Initialized");
}

void loop() {
  int flameDigital = digitalRead(FLAME_DO); // 0 or 1

  Serial.print(" | Flame Digital (DO): ");

  if (flameDigital == LOW) {
    Serial.println("🔥 FLAME DETECTED");
  } else {
    Serial.println("No Flame");
  }

  delay(500);
}