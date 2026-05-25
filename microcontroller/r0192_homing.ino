#include <SPI.h>
#include <mcp2515.h>

// --- Debug Konfiguration ---
const bool DEBUG_MODE = true; // Auf 'false' setzen, um alle seriellen Ausgaben zu deaktivieren

// --- Pin-Definitionen ESP-S3 ---
// (Hinweis: D3 und D0 verwendet, damit das XIAO ESP32-S3 Pinout korrekt gemappt wird)
//const int SPI_CS_PIN = D2;      
//const int HALL_SENSOR_PIN = D3; 

// --- Pin-Definitionen für Arduino Uno R3 ---
const int SPI_CS_PIN = 10;      // Chip Select für MCP2515 (Pin 10)
const int HALL_SENSOR_PIN = 3;  // Signal-Pin für TLE4935L (Pin 3)

// --- CAN-Bus Konfiguration ---
MCP2515 mcp2515(SPI_CS_PIN);
struct can_frame canMsg;

// Definieren Sie hier die CAN-IDs für Ihr System
const uint32_t CMD_HOST_ID = 0x00;   // ID vom Raspberry Pi --> zurück zum Pi
const uint32_t MSG_SLAVE_ID = 0x100; // ID vom Homing Microcontroller --> Empfangen vom Pi

// --- Homing Status ---
bool isHomingActive = false;

// --- Hilfsfunktion für den Debug-Modus ---
// Druckt CAN-Nachrichten formatiert im Seriellen Monitor
void printCanFrame(const char* direction, struct can_frame* frame) {
  if (!DEBUG_MODE) return;
  
  Serial.print(direction);
  Serial.print(" | ID: 0x");
  Serial.print(frame->can_id, HEX);
  Serial.print(" | DLC: ");
  Serial.print(frame->can_dlc);
  Serial.print(" | Data: ");
  for (int i = 0; i < frame->can_dlc; i++) {
    Serial.print("0x");
    if (frame->data[i] < 0x10) Serial.print("0"); // Führende Null für saubere Optik (z.B. 0x0F)
    Serial.print(frame->data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  if (DEBUG_MODE) {
    Serial.begin(115200);
    // while (!Serial); // Optional einkommentieren, wenn man den Boot-Vorgang sehen MUSS
  }

  // --- Hall-Sensor Setup ---
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

  // --- CAN-Bus Setup ---
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); 
  mcp2515.setNormalMode();

  if (DEBUG_MODE) {
    Serial.println("==========================================");
    Serial.println("R0192 Homing-Node gestartet.");
    Serial.println("Warte auf Homing-Befehl via CAN...");
    Serial.println("==========================================");
  }
}

void loop() {
  // 1. CAN-Bus Nachrichten kontinuierlich abfragen
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    
    // Debug-Ausgabe JEDER empfangenen Nachricht
    printCanFrame("RX", &canMsg);
    
    // Prüfen, ob die Nachricht der Homing-Befehl vom Raspberry (ROS 2) ist
    if (canMsg.can_id == MSG_SLAVE_ID) {
      if (DEBUG_MODE) Serial.println(">>> Homing-Befehl empfangen! Starte Überwachung des Hall-Sensors.");
      isHomingActive = true;
    }
  }

  // 2. Homing-Prozess (wird nur ausgeführt, wenn isHomingActive == true)
  if (isHomingActive) {
    // TLE4935L schaltet auf LOW, wenn der Magnet erkannt wird
    if (digitalRead(HALL_SENSOR_PIN) == LOW) {
      if (DEBUG_MODE) Serial.println(">>> Nullpunkt (Magnet) detektiert! Sende Bestätigung...");

      // Bestätigungsnachricht vorbereiten
      struct can_frame successMsg;
      successMsg.can_id = CMD_HOST_ID;
      successMsg.can_dlc = 1;      // Wir senden 1 Byte Daten
      successMsg.data[0] = 0xFF;   // Bestätigungscode

      // Nachricht senden
      if (mcp2515.sendMessage(&successMsg) == MCP2515::ERROR_OK) {
        if (DEBUG_MODE) Serial.println("Erfolgsmeldung via CAN gesendet.");
        
        // Debug-Ausgabe der gesendeten Nachricht
        printCanFrame("TX", &successMsg);
        
        isHomingActive = false; // Homing beendet, zurück in den Wartezustand
      } else {
        if (DEBUG_MODE) Serial.println("FEHLER beim Senden der CAN-Nachricht!");
      }
      
      // Kleines Delay zum Entprellen / Verhindern von Spam
      delay(5); 
    }
  }
}