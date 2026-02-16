/*********************************************************************
 * ESP32-WROOM-32D SPI SLAVE + WiFi + ESP-NOW GATEWAY
 * Author: Nguyen Trung Nhan
 * Tương thích hoàn toàn với STM32F411RE SPI DMA Master (4 byte)
 * - Nhận 4 byte → echo lại mỗi byte +1
 * - Gửi dữ liệu nhận được qua WiFi (ưu tiên) hoặc ESP-NOW
 * - Auto reconnect WiFi + fallback ESP-NOW
 * - Non-blocking + debug rõ ràng
 *********************************************************************/

#include <ESP32SPISlave.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ================= SPI CONFIG ========================
ESP32SPISlave slave;
static constexpr size_t BUFFER_SIZE = 4;  // Phải khớp với STM32 (4 byte)

uint8_t spi_rx_buf[BUFFER_SIZE];   // Dữ liệu nhận từ STM32
uint8_t spi_tx_buf[BUFFER_SIZE];   // Dữ liệu trả về cho STM32 (echo +1)

bool spiReady = false;
unsigned long lastSpiMsg = 0;
#define SPI_STATUS_INTERVAL 5000

// ================= WIFI CONFIG =======================
const char* ssid = "Do Quyen";
const char* pass = "Putin2701";
bool wifiConnected = false;
unsigned long lastWifiCheck = 0;
unsigned long lastWifiRetry = 0;
#define WIFI_CHECK_INTERVAL 10000
#define WIFI_RETRY_INTERVAL 5000

// LED status (LED 2 nhấp nháy khi mất WiFi)
#define LED_PIN 2
unsigned long wifiBlinkTimer = 0;
bool ledState = false;
#define WIFI_BLINK_INTERVAL 300

// ================= ESP-NOW CONFIG ====================
#define MY_NODE_ID 1
uint8_t peerMAC[] = {0xE0, 0x72, 0xA1, 0xD7, 0xFC, 0xD4};  // MAC của ESP32-S3

typedef struct {
  uint8_t senderID;
  uint16_t packetID;
  unsigned long timestamp;
  bool pong;
  uint8_t payload[BUFFER_SIZE];
} Message;

Message msg;
volatile bool pongReceived = false;
uint16_t txPacket = 0;
bool espNowReady = false;

// ================= WIFI EVENT HANDLER ================
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[Author: Nguyen Trung Nhan][WiFi] Starting...");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("[Author: Nguyen Trung Nhan][WiFi] Connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("[Author: Nguyen Trung Nhan][WiFi] GOT IP: " + WiFi.localIP().toString());
      wifiConnected = true;
      digitalWrite(LED_PIN, HIGH);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[Author: Nguyen Trung Nhan][WiFi] DISCONNECTED");
      wifiConnected = false;
      break;
    default:
      break;
  }
}

// ================= WIFI MANAGER ======================
void connectWiFi() {
  if (millis() - lastWifiRetry < WIFI_RETRY_INTERVAL) return;
  lastWifiRetry = millis();
  Serial.println("[Author: Nguyen Trung Nhan][WiFi] Attempting connection...");
  WiFi.disconnect(true);
  WiFi.begin(ssid, pass);
}

void checkWiFi() {
  if (millis() - lastWifiCheck < WIFI_CHECK_INTERVAL) return;
  lastWifiCheck = millis();
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
  } else {
    wifiConnected = false;
    connectWiFi();
  }
}

void wifiStatusTasks() {
  if (!wifiConnected) {
    if (millis() - wifiBlinkTimer > WIFI_BLINK_INTERVAL) {
      wifiBlinkTimer = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}

// ================= ESP-NOW CALLBACKS =================
void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[ESP-NOW] TX OK" : "[ESP-NOW] TX FAIL");
}

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Message)) return;
  Message in;
  memcpy(&in, data, sizeof(in));
  Serial.println("[Author: Nguyen Trung Nhan][ESP-NOW] RX");
  if (in.pong) {
    pongReceived = true;
    Serial.println("[ESP-NOW] PONG RECEIVED");
    return;
  }
  // Trả pong
  Message pong;
  pong.senderID = MY_NODE_ID;
  pong.packetID = in.packetID;
  pong.timestamp = in.timestamp;
  pong.pong = true;
  esp_now_send(info->src_addr, (uint8_t*)&pong, sizeof(pong));
}

// ================= ESP-NOW INIT ======================
void setupESPNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[Author: Nguyen Trung Nhan][ESP-NOW] INIT FAIL");
    return;
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMAC, 6);
  peer.channel = 1;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  espNowReady = true;
  Serial.println("[ESP-NOW] READY");
}

// ================= SPI INIT ==========================
void setupSPI() {
  Serial.println("[Author: Nguyen Trung Nhan][SPI] Initializing slave (Mode 3)...");
  slave.setDataMode(SPI_MODE3);           // Quan trọng: khớp với STM32 (CPOL=1, CPHA=1)
  slave.setQueueSize(1);
  bool ok = slave.begin(HSPI, 18, 19, 23, 5);  // SCK=18, MISO=19, MOSI=23, CS=5
  if (!ok) {
    Serial.println("[SPI] INIT FAILED!");
    while (1);
  }
  memset(spi_tx_buf, 0, BUFFER_SIZE);
  spiReady = true;
  Serial.println("[SPI] READY - Waiting for STM32 Master...");
}

// ================= ROUTING LAYER =====================
void wirelessSend(uint8_t *payload) {
  memcpy(msg.payload, payload, BUFFER_SIZE);
  msg.senderID = MY_NODE_ID;
  msg.packetID = txPacket++;
  msg.timestamp = millis();
  msg.pong = false;

  if (wifiConnected) {
    Serial.println("[Author: Nguyen Trung Nhan][ROUTE] SEND → WiFi (not implemented yet - add your code)");
  } else if (espNowReady) {
    Serial.println("[ROUTE] SEND → ESP-NOW");
    esp_err_t r = esp_now_send(peerMAC, (uint8_t*)&msg, sizeof(msg));
    if (r != ESP_OK) Serial.println("[ESP-NOW] SEND ERROR");
  }
}

// ================= SETUP =============================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-WROOM-32D GATEWAY START ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.onEvent(WiFiEvent);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  setupSPI();
  connectWiFi();
  setupESPNow();

  Serial.println("SYSTEM READY");
}

// ================= MAIN LOOP =========================
void loop() {
  checkWiFi();
  wifiStatusTasks();

  // Xử lý SPI Slave transaction (non-blocking)
  size_t received = slave.transfer(spi_tx_buf, spi_rx_buf, BUFFER_SIZE);

  if (received == BUFFER_SIZE) {
    // In dữ liệu nhận được từ STM32
    Serial.print("[Author: Nguyen Trung Nhan][SPI] RX from STM32: ");
    for (int i = 0; i < BUFFER_SIZE; i++) {
      Serial.printf("%02X ", spi_rx_buf[i]);
    }
    Serial.println();

    // Echo logic: trả lại mỗi byte + 1 (khớp với kiểm tra của STM32)
    for (int i = 0; i < BUFFER_SIZE; i++) {
      spi_tx_buf[i] = spi_rx_buf[i] + 1;
    }

    // Gửi dữ liệu nhận được qua wireless
    wirelessSend(spi_rx_buf);

    Serial.println("[SPI] TX Echo sent (+1)");
  }
  else if (received > 0) {
    Serial.println("[Author: Nguyen Trung Nhan][SPI] Partial transaction! (" + String(received) + " bytes)");
  }

  // In trạng thái SPI định kỳ
  if (millis() - lastSpiMsg > SPI_STATUS_INTERVAL) {
    lastSpiMsg = millis();
    if (spiReady) {
      Serial.println("[Author: Nguyen Trung Nhan][SPI] Alive & waiting STM32...");
    }
  }

  delay(5);  // Nhỏ để không block loop
}