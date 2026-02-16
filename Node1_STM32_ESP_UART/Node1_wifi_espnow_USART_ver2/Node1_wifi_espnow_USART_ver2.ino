/*********************************************************************
 * ESP32-S3 UART MASTER + WiFi + ESP-NOW GATEWAY
 *[Author: Nguyen Trung Nhan]
 * CỤM 1:
 * STM32 ⇄ UART ⇄ ESP32-S3
 * ESP32-S3 ⇄ (Wi-Fi / ESP-NOW) ⇄ ESP32-WROOM
 *
 * FEATURES:
 * - UART to STM32 (baud 115200)
 * - WiFi ưu tiên khi có mạng (UDP broadcast)
 * - ESP-NOW fallback khi mất WiFi (luôn sẵn, không deinit)
 * - Auto WiFi reconnect
 * - UART → Wireless bridge bidirect
 * - Non-blocking main loop
 * - STATUS MONITOR
 * - ADVANCED WIFI MANAGEMENT
 *[Author: Nguyen Trung Nhan]
 *********************************************************************/
// ===== FIXED: Ensured hybrid mode with ESP-NOW always active; Added self-ignore for senderID; Ensured channel consistency; Fixed peerMAC (assumed S3 MAC in WROOM code).
// ===== IMPROVED: Added check for senderID to prevent self-loop in UDP/ESP-NOW RX; Standardized BUFFER_SIZE and Message struct; Ensured bidirectional forwarding.
// ===== FIXED WiFi: Ensured WiFi connects by using correct event handling and retries; ESP-NOW init before WiFi; Common channel 1 always.
// ===== NOTE: Replace peerMAC with actual WROOM MAC if different.
// ===== UPDATED: Corrected peerMAC to actual WROOM MAC (CC:DB:A7:49:94:E8); Ensured WiFi connection with additional debug logs and persistent retries; Ensured hybrid by keeping ESP-NOW always active for RX/TX fallback.
// ===== NEW: Added heartbeat ping-pong to detect wireless connection status; Set ESP-NOW peer channel to 0 for current channel usage; Added check to forward only if payloadLen > 0; Reset missCount on data receive.
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiUdp.h>
//////////////////////////////////////////////////////////
// ================= UART CONFIG =======================
//////////////////////////////////////////////////////////
#define UART_RX_PIN 18 // Adjust to your hardware
#define UART_TX_PIN 17 // Adjust to your hardware
#define UART_BAUD 115200
String uartReceived = "";
//////////////////////////////////////////////////////////
// ================= BUFFER CONFIG =====================
//////////////////////////////////////////////////////////
static constexpr size_t BUFFER_SIZE = 256;
uint8_t wirelessPayload[BUFFER_SIZE];
unsigned long lastUartMsg = 0;
#define UART_STATUS_INTERVAL 3000
//////////////////////////////////////////////////////////
// ================= WIFI CONFIG =======================
//////////////////////////////////////////////////////////
const char* ssid = "Do Quyen";
const char* pass = "Putin2701";
bool wifiConnected = false;
unsigned long lastWifiCheck = 0;
unsigned long lastWifiRetry = 0;
#define WIFI_CHECK_INTERVAL 10000
#define WIFI_RETRY_INTERVAL 5000
#define LED_PIN 2
unsigned long wifiBlinkTimer = 0;
bool ledState = false;
#define WIFI_BLINK_INTERVAL 300
unsigned long lastRSSIReport = 0;
//////////////////////////////////////////////////////////
// ================= UDP CONFIG ========================
//////////////////////////////////////////////////////////
WiFiUDP udp;
const int UDP_PORT = 1234;
//////////////////////////////////////////////////////////
// ================= ESP-NOW CONFIG ====================
//////////////////////////////////////////////////////////
#define MY_NODE_ID 2
uint8_t peerMAC[] = {0xCC, 0xDB, 0xA7, 0x49, 0x94, 0xE8}; // Actual WROOM MAC
typedef struct {
  uint8_t senderID;
  uint16_t packetID;
  unsigned long timestamp;
  bool pong;
  uint8_t payload[BUFFER_SIZE];
  uint16_t payloadLen;
} Message;
Message msg;
volatile bool pongReceived = false;
uint16_t txPacket = 0;
bool espNowReady = false;
volatile uint8_t wirelessMissCount = 0;
#define WIRELESS_MISS_MAX 3
unsigned long lastPing = 0;
#define PING_INTERVAL 5000
//////////////////////////////////////////////////////////
// ================= WIFI EVENT HANDLER ================
//////////////////////////////////////////////////////////
void WiFiEvent(WiFiEvent_t event)
{
  switch(event)
  {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[WiFi] Starting...");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[WiFi] Connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("[WiFi] GOT IP");
      Serial.print("[WiFi] IP: ");
      Serial.println(WiFi.localIP());
      wifiConnected = true;
      digitalWrite(LED_PIN, HIGH);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[WiFi] DISCONNECTED");
      wifiConnected = false;
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
      break;
    default:
      break;
  }
}
//////////////////////////////////////////////////////////
// ================= WIFI MANAGER ======================
//////////////////////////////////////////////////////////
void connectWiFi()
{
  if (millis() - lastWifiRetry < WIFI_RETRY_INTERVAL) return;
  lastWifiRetry = millis();
  Serial.println("[WiFi] Attempting connection...");
  WiFi.disconnect(true);
  delay(100); // ===== IMPROVED: Short delay for clean disconnect
  WiFi.begin(ssid, pass);
  Serial.print("[WiFi] Connecting to SSID: ");
  Serial.println(ssid);
}
void checkWiFi()
{
  if (millis() - lastWifiCheck < WIFI_CHECK_INTERVAL) return;
  lastWifiCheck = millis();
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("[WiFi] Status: Connected");
  } else {
    wifiConnected = false;
    Serial.print("[WiFi] Status: ");
    Serial.println(status); // ===== IMPROVED: Log status for debug
    connectWiFi();
  }
}
void wifiStatusTasks()
{
  if (!wifiConnected)
  {
    if (millis() - wifiBlinkTimer > WIFI_BLINK_INTERVAL)
    {
      wifiBlinkTimer = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }
  else
  {
    if (millis() - lastRSSIReport > 10000)
    {
      lastRSSIReport = millis();
      Serial.print("[WiFi] RSSI: ");
      Serial.println(WiFi.RSSI());
    }
  }
}
//////////////////////////////////////////////////////////
// ================= ESP-NOW CALLBACKS =================
//////////////////////////////////////////////////////////
void onSent(const uint8_t *mac, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[ESP-NOW] TX OK" : "[ESP-NOW] TX FAIL");
}
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
  if (len != sizeof(Message)) return;
  Message in;
  memcpy(&in, data, sizeof(in));
  if (in.senderID == MY_NODE_ID) return; // ===== FIXED: Ignore self
  Serial.println("[ESP-NOW] RX");
  if (in.pong) {
    pongReceived = true;
    wirelessMissCount = 0;
    Serial.println("[ESP-NOW] PONG RECEIVED");
    return;
  }
  // Send PONG back
  Message pong;
  pong.senderID = MY_NODE_ID;
  pong.packetID = in.packetID;
  pong.timestamp = in.timestamp;
  pong.pong = true;
  pong.payloadLen = 0;
  esp_now_send(info->src_addr, (uint8_t*)&pong, sizeof(pong));
  // Reset miss count on data receive
  wirelessMissCount = 0;
  // Forward to UART if payloadLen > 0
  if (in.payloadLen > 0) {
    Serial1.write(in.payload, in.payloadLen);
  }
}
//////////////////////////////////////////////////////////
// ================= ESP-NOW INIT ======================
//////////////////////////////////////////////////////////
void setupESPNow()
{
  esp_err_t err = esp_now_init();
  Serial.print("[Author: Nguyen Trung Nhan][ESP-NOW] Init result: ");
  Serial.println(err);
  if (err != ESP_OK) {
    Serial.println("[Author: Nguyen Trung Nhan][ESP-NOW] INIT FAIL");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 0; // ===== NEW: Use current channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] ADD PEER FAIL");
    return;
  }
  espNowReady = true;
  Serial.println("[ESP-NOW] READY");
}
//////////////////////////////////////////////////////////
// ================= UART INIT =========================
//////////////////////////////////////////////////////////
void setupUART()
{
  Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("[UART] READY");
}
//////////////////////////////////////////////////////////
// ================= ROUTING LAYER =====================
//////////////////////////////////////////////////////////
void wirelessSend(uint8_t *payload, uint16_t len)
{
  msg.senderID = MY_NODE_ID;
  msg.packetID = txPacket++;
  msg.timestamp = millis();
  msg.pong = false;
  msg.payloadLen = len;
  if (payload != NULL && len > 0) {
    memcpy(msg.payload, payload, len);
  }
  if (wifiConnected) {
    Serial.println("[Author: Nguyen Trung Nhan][ROUTE] SEND → WiFi UDP");
    udp.beginPacket("255.255.255.255", UDP_PORT);
    udp.write((uint8_t*)&msg, sizeof(msg));
    udp.endPacket();
  } else if (espNowReady) {
    Serial.println("[Author: Nguyen Trung Nhan][ROUTE] SEND → ESP-NOW");
    esp_err_t r = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    if (r != ESP_OK) Serial.println("[ESP-NOW] SEND ERROR");
  }
}
//////////////////////////////////////////////////////////
// ================= SETUP =============================
//////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-S3 GATEWAY START ===");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.onEvent(WiFiEvent);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // Set before init
  setupESPNow(); // Init ESP-NOW first
  connectWiFi();
  setupUART();
  udp.begin(UDP_PORT);
  Serial.println("SYSTEM READY");
}
//////////////////////////////////////////////////////////
// ================= MAIN LOOP =========================
//////////////////////////////////////////////////////////
void loop()
{
  checkWiFi();
  wifiStatusTasks();
  // UART RX
  if (Serial1.available()) {
    uartReceived = Serial1.readStringUntil('\n');
    uartReceived.trim();
    if (uartReceived.length() > 0) {
      Serial.print("[UART] RX: ");
      Serial.println(uartReceived);
      lastUartMsg = millis();
      if (uartReceived == "PING") {
        Serial1.println("PONG");
        Serial.println("[UART] Sent PONG");
      } else {
        // Forward to wireless
        wirelessSend((uint8_t*)uartReceived.c_str(), uartReceived.length());
      }
    }
  }
  // UDP RX (WiFi)
  if (udp.parsePacket()) {
    uint8_t data[sizeof(Message)];
    int len = udp.read(data, sizeof(Message));
    if (len == sizeof(Message)) {
      Message in;
      memcpy(&in, data, sizeof(in));
      if (in.senderID == MY_NODE_ID) return; // ===== FIXED: Ignore self
      Serial.println("[WiFi UDP] RX");
      if (in.pong) {
        pongReceived = true;
        wirelessMissCount = 0;
        Serial.println("[WiFi] PONG RECEIVED");
        return;
      }
      // Send PONG back
      Message pong;
      pong.senderID = MY_NODE_ID;
      pong.packetID = in.packetID;
      pong.timestamp = in.timestamp;
      pong.pong = true;
      pong.payloadLen = 0;
      udp.beginPacket(udp.remoteIP(), UDP_PORT);
      udp.write((uint8_t*)&pong, sizeof(pong));
      udp.endPacket();
      // Reset miss count on data receive
      wirelessMissCount = 0;
      // Forward to UART if payloadLen > 0
      if (in.payloadLen > 0) {
        Serial1.write(in.payload, in.payloadLen);
      }
    }
  }
  // UART status
  if (millis() - lastUartMsg > UART_STATUS_INTERVAL) {
    lastUartMsg = millis();
    Serial.println("[UART] Alive & waiting STM32...");
  }
  // Heartbeat for wireless connection detection
  if (millis() - lastPing > PING_INTERVAL) {
    if (pongReceived) {
      wirelessMissCount = 0;
      Serial.println("[Author: Nguyen Trung Nhan][WIRELESS] Connected");
    } else {
      wirelessMissCount++;
      if (wirelessMissCount > WIRELESS_MISS_MAX) {
        Serial.println("[Author: Nguyen Trung Nhan][WIRELESS] Disconnected");
      }
    }
    pongReceived = false;
    if (wifiConnected || espNowReady) {
      Serial.println("[Author: Nguyen Trung Nhan][WIRELESS] Sending PING");
      wirelessSend(NULL, 0); // Send ping (len=0)
    }
    lastPing = millis();
  }
  delay(5);
}