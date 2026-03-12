#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// SD Card SPI Pin Configuration
#define SD_CS    10
#define SD_MOSI  11
#define SD_SCK   12
#define SD_MISO  13

// UART Configuration
#define UART_TX  17
#define UART_RX  18
#define UART_BAUD 115200

// File config
#define LOG_FILE "/uart_log.txt"

File logFile;
String buffer = "";

void initSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  
  Serial.print("Initializing SD card...");
  
  if (!SD.begin(SD_CS)) {
    Serial.println("FAILED! Check wiring.");
    while (true) delay(1000);
  }
  
  Serial.println("SD card initialized successfully.");
  
  // Write header/boot marker
  logFile = SD.open(LOG_FILE, FILE_APPEND);
  if (logFile) {
    logFile.println("\n--- Device Boot: " + String(millis()) + "ms ---");
    logFile.close();
    Serial.println("Log file ready: " + String(LOG_FILE));
  } else {
    Serial.println("ERROR: Could not open log file!");
  }
}

void saveToSD(const String& data) {
  logFile = SD.open(LOG_FILE, FILE_APPEND);
  
  if (logFile) {
    logFile.println(data);
    logFile.close();
    Serial.println("[SAVED] " + data);
  } else {
    Serial.println("[ERROR] Failed to open file for writing!");
  }
}

void setup() {
  // Debug serial (USB)
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 UART to SD Logger Starting...");

  // UART2 for receiving data
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println("UART2 initialized on TX:17, RX:18");

  // Initialize SD card
  initSD();

  Serial.println("System ready. Waiting for UART data...");
}

void loop() {
  // Read all available UART data
  while (Serial2.available()) {
    char c = Serial2.read();
    
    if (c == '\n' || c == '\r') {
      // Save complete line when newline received
      if (buffer.length() > 0) {
        buffer.trim();
        saveToSD(buffer);
        buffer = "";
      }
    } else {
      buffer += c;
      
      // Safety: save if buffer gets too large
      if (buffer.length() >= 512) {
        saveToSD(buffer);
        buffer = "";
      }
    }
  }
}