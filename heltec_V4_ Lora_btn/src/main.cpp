#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <U8g2lib.h>

#ifndef NODE_ID
#define NODE_ID 1
#endif
static_assert(NODE_ID == 1 || NODE_ID == 2, "NODE_ID must be 1 or 2");

// Heltec WiFi LoRa 32 V4.3, not V3 or V4-R8.
constexpr int PIN_NSS = 8, PIN_SCK = 9, PIN_MOSI = 10, PIN_MISO = 11;
constexpr int PIN_RST = 12, PIN_BUSY = 13, PIN_DIO1 = 14;
constexpr int PIN_FEM_CSD = 2, PIN_FEM_CTX = 5, PIN_VFEM = 7;
constexpr int PIN_OLED_SDA = 17, PIN_OLED_SCL = 18, PIN_OLED_RST = 21, PIN_VEXT = 36;
constexpr int PIN_BUTTON = 4;       // Node A: button from GPIO4 to GND.
constexpr int PIN_EXTERNAL_LED = 4; // Node B: GPIO4 -> resistor -> LED -> GND.
constexpr int PIN_ONBOARD_LED = 35; // Node B: built-in white LED.

constexpr float FREQ_MHZ = 868.0;
constexpr uint32_t DEBOUNCE_MS = 120, ACK_TIMEOUT_MS = 2200;
constexpr uint32_t HEARTBEAT_MS = 15000, RETRY_MS = 5000, LINK_TIMEOUT_MS = 45000;
constexpr uint8_t MAX_ATTEMPTS = 3;

SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);
volatile bool radioEvent = false;
void IRAM_ATTR onRadioEvent() { radioEvent = true; }

bool rawButton = false, stableButton = false;
uint32_t rawChangedMs = 0, bootToken = 0, nextSequence = 1, pendingSequence = 0;
uint32_t buttonGeneration = 0, pendingGeneration = 0;
bool pendingState = false, awaitingAck = false, confirmed = false, confirmedState = false;
bool stateDirty = true;
uint8_t attempts = 0;
uint32_t lastSendMs = 0, lastConfirmedMs = 0, nextSendMs = 0;

bool ledState = false, havePacket = false;
uint32_t lastPacketMs = 0, peerBoot = 0, peerSequence = 0;

void showStatus(const char* title, const char* line1, const char* line2 = "") {
  display.clearBuffer();
  display.setFont(u8g2_font_6x13_tr);
  display.drawStr(0, 12, title);
  display.drawHLine(0, 16, 128);
  display.drawStr(0, 34, line1);
  display.drawStr(0, 51, line2);
  display.sendBuffer();
}

void showSender() {
  showStatus("NODE A: BUTTON", stableButton ? "Button: ON" : "Button: OFF",
             awaitingAck ? "Sending..." :
             confirmed && !stateDirty && stableButton == confirmedState &&
             millis() - lastConfirmedMs < LINK_TIMEOUT_MS ? "Link: OK" : "Link: waiting");
}

void setLight(bool on) {
  ledState = on;
  digitalWrite(PIN_EXTERNAL_LED, on ? HIGH : LOW);
  digitalWrite(PIN_ONBOARD_LED, on ? HIGH : LOW);
  showStatus("NODE B: LIGHT", on ? "LED: ON" : "LED: OFF",
             havePacket ? "Radio: receiving" : "Radio: waiting");
}

void enterRx() {
  digitalWrite(PIN_FEM_CTX, LOW); // RX through LNA on V4.3.
  radioEvent = false;
  int rc = radio.startReceive();
  if (rc != RADIOLIB_ERR_NONE) Serial.printf("[RX ERROR] %d\n", rc);
}

void transmitPacket(String packet) {
  radio.standby();
  digitalWrite(PIN_FEM_CTX, HIGH); // TX through PA on V4.3.
  delay(2);
  int rc = radio.transmit(packet);
  digitalWrite(PIN_FEM_CTX, LOW);
  if (rc != RADIOLIB_ERR_NONE) Serial.printf("[TX ERROR] %d\n", rc);
  enterRx();
}

void sendAttempt() {
  ++attempts;
  String packet = "S|" + String(bootToken) + "|" + String(pendingSequence) +
                  "|" + (pendingState ? "1" : "0");
  Serial.printf("[TX] button=%u seq=%lu attempt=%u/%u\n", pendingState,
                (unsigned long)pendingSequence, attempts, MAX_ATTEMPTS);
  transmitPacket(packet);
  lastSendMs = millis();
  showSender();
}

void startStatePacket() {
  pendingState = stableButton;
  pendingGeneration = buttonGeneration;
  pendingSequence = nextSequence++;
  attempts = 0;
  awaitingAck = true;
  sendAttempt();
}

bool parseNumber(const String& s, uint32_t& value) {
  if (s.length() == 0) return false;
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
  }
  value = strtoul(s.c_str(), nullptr, 10);
  return true;
}

void handlePacket(const String& packet, float rssi, float snr) {
  int first = packet.indexOf('|');
  if (first < 0) return;
  int second = packet.indexOf('|', first + 1);
  if (second < 0) return;

  if (NODE_ID == 1) {
    if (packet.substring(0, first) != "A" || packet.indexOf('|', second + 1) >= 0) return;
    uint32_t ackBoot, ackSequence;
    if (!parseNumber(packet.substring(first + 1, second), ackBoot) ||
        !parseNumber(packet.substring(second + 1), ackSequence)) return;
    if (awaitingAck && ackBoot == bootToken && ackSequence == pendingSequence) {
      awaitingAck = false;
      confirmed = true;
      confirmedState = pendingState;
      if (pendingGeneration == buttonGeneration && pendingState == stableButton) stateDirty = false;
      lastConfirmedMs = nextSendMs = millis();
      Serial.printf("[ACK] button=%u seq=%lu RSSI=%.1f SNR=%.1f\n",
                    confirmedState, (unsigned long)ackSequence, rssi, snr);
      showSender();
    }
    return;
  }

  if (packet.substring(0, first) != "S") return;
  int third = packet.indexOf('|', second + 1);
  if (third < 0 || packet.indexOf('|', third + 1) >= 0) return;
  uint32_t senderBoot, sequence;
  if (!parseNumber(packet.substring(first + 1, second), senderBoot) ||
      !parseNumber(packet.substring(second + 1, third), sequence)) return;
  String state = packet.substring(third + 1);
  if (state != "0" && state != "1") return;

  bool newBoot = !havePacket || senderBoot != peerBoot;
  bool newer = newBoot || (int32_t)(sequence - peerSequence) > 0;
  bool duplicate = !newBoot && sequence == peerSequence;
  if (newer || duplicate) {
    havePacket = true;
    lastPacketMs = millis();
    if (newer) {
      peerBoot = senderBoot;
      peerSequence = sequence;
      setLight(state == "1");
      Serial.printf("[RX] button=%s seq=%lu RSSI=%.1f SNR=%.1f\n",
                    state.c_str(), (unsigned long)sequence, rssi, snr);
    }
  }
  // ACK duplicates too: the earlier ACK may have been lost.
  transmitPacket("A|" + String(senderBoot) + "|" + String(sequence));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.printf("\nHeltec V4.3 button -> LED, node %d\n", NODE_ID);
  if (NODE_ID == 1) {
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    rawButton = stableButton = digitalRead(PIN_BUTTON) == LOW;
  } else {
    pinMode(PIN_EXTERNAL_LED, OUTPUT);
    pinMode(PIN_ONBOARD_LED, OUTPUT);
    digitalWrite(PIN_EXTERNAL_LED, LOW);
    digitalWrite(PIN_ONBOARD_LED, LOW);
  }
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW); // OLED power is active low.
  delay(100);
  display.begin();
  showStatus(NODE_ID == 1 ? "NODE A: BUTTON" : "NODE B: LIGHT", "Starting LoRa...");
  pinMode(PIN_VFEM, OUTPUT); digitalWrite(PIN_VFEM, HIGH);
  pinMode(PIN_FEM_CSD, OUTPUT); digitalWrite(PIN_FEM_CSD, HIGH);
  pinMode(PIN_FEM_CTX, OUTPUT); digitalWrite(PIN_FEM_CTX, LOW);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  int rc = radio.begin(FREQ_MHZ, 125.0, 9, 7, 0x12, 10, 8, 1.6, false);
  if (rc != RADIOLIB_ERR_NONE) {
    Serial.printf("[INIT FAILED] %d\n", rc);
    showStatus("LORA ERROR", "Radio init failed");
    while (true) delay(1000);
  }
  // PA_CPS is connected to SX1262 DIO2 on V4.3.
  rc = radio.setDio2AsRfSwitch(true);
  if (rc != RADIOLIB_ERR_NONE) {
    Serial.printf("[DIO2 ERROR] %d\n", rc);
    showStatus("LORA ERROR", "RF switch failed");
    while (true) delay(1000);
  }
  radio.setCRC(true);
  radio.setDio1Action(onRadioEvent);
  bootToken = esp_random();
  enterRx();
  if (NODE_ID == 1) showSender(); else setLight(false);
}

void loop() {
  if (NODE_ID == 1) {
    bool raw = digitalRead(PIN_BUTTON) == LOW;
    if (raw != rawButton) {
      rawButton = raw;
      rawChangedMs = millis();
    }
    if (raw != stableButton && millis() - rawChangedMs >= DEBOUNCE_MS) {
      stableButton = raw;
      ++buttonGeneration;
      stateDirty = true;
      Serial.printf("[BUTTON] %s\n", stableButton ? "ON" : "OFF");
      if (awaitingAck && stableButton != pendingState) awaitingAck = false;
      nextSendMs = millis();
      showSender();
    }
  }

  if (radioEvent) {
    radioEvent = false;
    String packet;
    int rc = radio.readData(packet);
    if (rc == RADIOLIB_ERR_NONE) {
      float rssi = radio.getRSSI(), snr = radio.getSNR();
      enterRx();
      handlePacket(packet, rssi, snr);
    } else enterRx();
  }

  if (NODE_ID == 1) {
    uint32_t now = millis();
    if (awaitingAck && now - lastSendMs >= ACK_TIMEOUT_MS) {
      if (attempts < MAX_ATTEMPTS) sendAttempt();
      else {
        Serial.printf("[NO ACK] seq=%lu; retry later\n", (unsigned long)pendingSequence);
        awaitingAck = false;
        nextSendMs = millis() + RETRY_MS;
        showSender();
      }
    }
    bool updateNeeded = stateDirty || !confirmed || stableButton != confirmedState ||
                        now - lastConfirmedMs >= HEARTBEAT_MS;
    if (!awaitingAck && updateNeeded && (int32_t)(now - nextSendMs) >= 0) startStatePacket();
  } else if (havePacket && millis() - lastPacketMs >= LINK_TIMEOUT_MS) {
    havePacket = false;
    setLight(false);
    Serial.println("[LINK LOST] LED off");
  }
}
