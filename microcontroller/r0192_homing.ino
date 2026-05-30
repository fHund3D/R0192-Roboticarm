#include <SPI.h>
#include <mcp2515.h>  //Library: autowp

// ============================================================================
// R0192 Homing-Node (ein Arduino pro Achse)
//
// Protokoll (Pi <-> Arduino) — EINE achsenspezifische CAN-ID für beide
// Richtungen, Bedeutung steckt in Data[0]:
//   Pi  -> Arduino : Data[0] = CMD_ARM      (0x01)  Sensor scharf stellen
//   Arduino -> Pi  : Data[0] = RSP_DETECTED (0xFF)  Magnet erkannt
//   Arduino -> Pi  : Data[0] = RSP_ERROR    (0xEE)  Timeout / kein Magnet
//
// Für Achsen 2-6 nur AXIS_CAN_ID anpassen (0x101 ... 0x105).
// ============================================================================

// --- Debug Konfiguration ---
const bool DEBUG_MODE = true; // Auf 'false' setzen, um alle seriellen Ausgaben zu deaktivieren

// --- Pin-Definitionen ESP-S3 ---
// (Hinweis: D3 und D0 verwendet, damit das XIAO ESP32-S3 Pinout korrekt gemappt wird)
//const int SPI_CS_PIN = D2;
//const int HALL_SENSOR_PIN = D3;

// --- Pin-Definitionen für Arduino Uno R3 ---
const int SPI_CS_PIN = 10;      // Chip Select für MCP2515 (Pin 10)
const int HALL_SENSOR_PIN = 3;  // Signal-Pin für TLE4905L (Pin 3)

// --- CAN-Bus Konfiguration ---
MCP2515 mcp2515(SPI_CS_PIN);
struct can_frame canMsg;

// Achsenspezifische CAN-ID (Achse 1 = 0x100). Wird für RX UND TX verwendet.
const uint32_t AXIS_CAN_ID = 0x100;

// Daten-Codes (Data[0])
const uint8_t CMD_ARM      = 0x01; // Pi -> Arduino: Sensor scharf stellen
const uint8_t RSP_DETECTED = 0xFF; // Arduino -> Pi: Magnet erkannt
const uint8_t RSP_ERROR    = 0xEE; // Arduino -> Pi: Timeout / Fehler

// Timeout, nach dem ohne Hall-Detektion abgebrochen wird (ms).
const unsigned long HOMING_TIMEOUT_MS = 30000;

// --- Homing Status ---
bool isHomingActive = false;
unsigned long homingStartMs = 0;  // Zeitstempel beim Scharfstellen

// --- Hilfsfunktion für den Debug-Modus ---
// Druckt nur Frames der EIGENEN AXIS_CAN_ID, damit der Monitor bei mehreren
// Arduinos am selben Bus lesbar bleibt.
void printCanFrame(const char* direction, struct can_frame* frame) {
  if (!DEBUG_MODE) return;
  if (frame->can_id != AXIS_CAN_ID) return;  // fremde Achsen ignorieren

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

// Sendet eine 1-Byte-Antwort auf AXIS_CAN_ID an den Pi.
void sendResponse(uint8_t code) {
  struct can_frame msg;
  msg.can_id  = AXIS_CAN_ID;
  msg.can_dlc = 1;
  msg.data[0] = code;

  if (mcp2515.sendMessage(&msg) == MCP2515::ERROR_OK) {
    printCanFrame("TX", &msg);
  } else if (DEBUG_MODE) {
    Serial.println("FEHLER beim Senden der CAN-Nachricht!");
  }
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
    Serial.print("R0192 Homing-Node gestartet. AXIS_CAN_ID = 0x");
    Serial.println(AXIS_CAN_ID, HEX);
    Serial.println("Warte auf CMD_ARM via CAN...");
    Serial.println("==========================================");
  }
}

void loop() {
  // 1. CAN-Bus Nachrichten kontinuierlich abfragen
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    // Debug-Ausgabe (nur eigene ID, gefiltert in printCanFrame)
    printCanFrame("RX", &canMsg);

    // Scharf stellen nur bei eigener ID UND korrektem Befehlscode
    if (canMsg.can_id == AXIS_CAN_ID && canMsg.can_dlc >= 1 && canMsg.data[0] == CMD_ARM) {
      if (DEBUG_MODE) Serial.println(">>> CMD_ARM empfangen! Starte Überwachung des Hall-Sensors.");
      isHomingActive = true;
      homingStartMs = millis();
    }
  }

  // 2. Homing-Prozess (wird nur ausgeführt, wenn isHomingActive == true)
  if (isHomingActive) {
    // TLE4905L schaltet auf LOW, wenn der Magnet erkannt wird
    if (digitalRead(HALL_SENSOR_PIN) == LOW) {
      if (DEBUG_MODE) Serial.println(">>> Nullpunkt (Magnet) detektiert! Sende RSP_DETECTED...");
      sendResponse(RSP_DETECTED);
      isHomingActive = false; // Homing beendet, zurück in den Wartezustand
      delay(5);               // kleines Delay zum Entprellen / Anti-Spam
    }
    // Timeout-Überwachung: verhindert Festhängen, wenn kein Hall-Signal kommt
    else if (millis() - homingStartMs >= HOMING_TIMEOUT_MS) {
      if (DEBUG_MODE) Serial.println(">>> TIMEOUT — kein Magnet erkannt! Sende RSP_ERROR...");
      sendResponse(RSP_ERROR);
      isHomingActive = false; // zurück in den Wartezustand (kein Hängenbleiben)
    }
  }
}
