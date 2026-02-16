## Describe this project
Adaptive Wireless Failover Architecture (AWFA) is a distributed two-cluster embedded system integrating real-time control units based on STM32 and wireless gateways based on ESP32. Each cluster consists of one STM32 and one ESP32. The STM32 handles deterministic sensor acquisition and time-critical control tasks, while the ESP32 manages wireless communication and inter-cluster data exchange. In Cluster 1, the STM32 communicates with an ESP32-S3 via UART; in Cluster 2, the STM32 interfaces with an ESP32-WROOM-32D via SPI. Sensor data is transferred from STM32 to ESP32 for packetization and wireless transmission to the remote cluster. The system implements a hybrid communication model where Wi-Fi (infrastructure mode) and ESP-NOW (peer-to-peer mode) operate concurrently on the 2.4 GHz band. Each ESP32 runs in Wi-Fi Station mode to connect to available network infrastructure while simultaneously maintaining an active ESP-NOW session. When Wi-Fi is available, data transmission is prioritized over TCP/UDP or suitable application protocols through the infrastructure network. ESP-NOW remains active in parallel, without entering sleep mode. If Wi-Fi becomes unavailable, traffic is automatically redirected to ESP-NOW without reinitialization delay, enabling instant failover and uninterrupted communication. This architecture forms a Hybrid Wireless Sensor Network where STM32 devices act as sensor/processing nodes and ESP32 modules function as communication gateways. By maintaining dual communication stacks in parallel, AWFA achieves high availability, adaptive redundancy, low latency, and robust operation under dynamic network conditions.

## Flowchart
```
                        ┌────────────────────────────┐
                        │        CLUSTER 1           │
                        │  ┌──────────────────────┐  │
Sensor/Data ───────────►│  │       STM32          │  │
                        │  │  - Sensor Processing │  │
                        │  │  - Real-time Control │  │
                        │  └──────────┬───────────┘  │
                        │             │ UART (TX/RX) │
                        │  ┌──────────▼───────────┐  │
                        │  │     ESP32-S3         │  │
                        │  │  - Packetization     │  │
                        │  │  - WiFi + ESP-NOW    │  │
                        │  └──────────┬───────────┘  │
                        └─────────────┼──────────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    │   Hybrid Wireless Communication   │
                    │  (WiFi Infrastructure + ESP-NOW)  │
                    └─────────────────┼─────────────────┘
                                      │
                        ┌─────────────┼──────────────┐
                        │        CLUSTER 2           │
                        │  ┌──────────▼───────────┐  │
                        │  │   ESP32-WROOM-32D    │  │
                        │  │  - WiFi + ESP-NOW    │  │
                        │  │  - Depacketization   │  │
                        │  └──────────┬───────────┘  │
                        │             │ SPI          │
                        │  ┌──────────▼───────────┐  │
                        │  │       STM32          │  │
                        │  │  - Data Processing   │  │
                        │  │  - Control Output    │  │
                        │  └──────────────────────┘  │
                        └────────────────────────────┘
```
## Communication Decision Flow (Failover Logic)
```
                ┌─────────────────────────┐
                │   Start Transmission    │
                └─────────────┬───────────┘
                              │
                     WiFi Connected?
                              │
                ┌─────────────┴─────────────┐
                │                           │
              YES                          NO
                │                           │
   Send via WiFi (TCP/UDP)       Send via ESP-NOW
                │                           │
   ESP-NOW remains active        (Already Active)
   in background (no sleep)                 │
                └─────────────┬─────────────┘
                              │
                     Continuous Monitoring
```
