/**
  ******************************************************************************
  * @file    main.cpp
  * @brief   Cubli-ESP: ESP32-WROOM-32E 无线上位机通信桥
  *
  * 功能:
  *   STM32F407ZGT6 (USART1, PA9/PA10) <--UART--> ESP32 (IO26=RX / IO25=TX)
  *   ESP32 通过 WiFi TCP 与电脑上位机通信, 完全取代原 USART2 有线串口。
  *
  *   - 上行: F407 发来的调试帧 ($TELE...#, 命令回显, 日志) 原样透传到所有 TCP 客户端
  *   - 下行: 上位机 TCP 发来的命令行 (TEST/BAL/M1.../TELE...) 原样透传给 F407
  *   - 协议字节流透明桥接, 上位机解析逻辑与有线串口完全一致
  *
  * WiFi 策略:
  *   1. 优先 STA 模式连接下方配置的路由器 (CUBLI_WIFI_SSID / CUBLI_WIFI_PASS)
  *   2. 连接失败 20s 后自动切换 AP 热点模式: SSID=Cubli-ESP, 密码=cubli12345,
  *      AP 模式下 ESP32 固定 IP = 192.168.4.1
  *   3. TCP 服务器端口 3333 (STA 与 AP 模式下均开启)
  *   上位机连接地址: STA 模式填路由器分配的 IP (上电时打印于 USB 串口监视器),
  *                   AP 模式填 192.168.4.1
  *
  * 硬件连线 (依据网表 Netlist_MB_Schematic_2026-10-07.net):
  *   ESP32 IO25 (TX) -> F407 PA10 (USART1_RX)
  *   ESP32 IO26 (RX) <- F407 PA9  (USART1_TX)
  *   波特率 115200-8-N-1 (与 F407 侧 MX_USART1_UART_Init 一致)
  ******************************************************************************
  */
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>

// ============================= 用户配置区 =============================
/* STA 模式路由器凭据: 修改为实际环境后即可自动连接家庭/实验室 WiFi */
#define CUBLI_WIFI_SSID     "Cubli-Lab"     /* 路由器名称 (无此路由器则自动进入AP模式) */
#define CUBLI_WIFI_PASS     "cubli12345"    /* 路由器密码 (至少8位) */

#define CUBLI_AP_SSID       "Cubli-ESP"     /* AP 模式热点名称 */
#define CUBLI_AP_PASS       "cubli12345"    /* AP 模式热点密码 (至少8位) */

#define CUBLI_TCP_PORT      3333            /* 上位机 TCP 连接端口 */

/* UART 桥接配置 (与 F407 MX_USART1_UART_Init 保持一致) */
#define FOC_UART_RX_PIN     26              /* ESP32 IO26 <- F407 PA9 (USART1_TX) */
#define FOC_UART_TX_PIN     25              /* ESP32 IO25 -> F407 PA10 (USART1_RX) */
#define FOC_UART_BAUD       115200

#define STATUS_LED_PIN      2               /* 板载 LED: 运行/连接状态指示 */
#define MAX_TCP_CLIENTS     1               /* 最多同时接入的上位机数量 */
// =====================================================================

WiFiServer tcp_server(CUBLI_TCP_PORT);
WiFiClient tcp_clients[MAX_TCP_CLIENTS];

/* 双向透传统计 (每 5s 打印一次, 便于排查无线链路) */
static uint32_t stat_uart_to_tcp = 0;
static uint32_t stat_tcp_to_uart = 0;
static uint32_t stat_last_print_ms = 0;
static uint32_t led_last_toggle_ms = 0;
static bool led_state = false;
static bool had_client = false;
static char command_line[256];
static size_t command_length=0;
static bool command_overflow=false;

/* ------------------------------------------------------------------ */
/* 释放已断开的 TCP 客户端槽位                                         */
static void cleanup_clients(void)
{
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcp_clients[i] && !tcp_clients[i].connected()) {
            tcp_clients[i].stop();
        }
    }
}

/* ------------------------------------------------------------------ */
/* 接入新的 TCP 客户端                                                 */
static void accept_clients(void)
{
    WiFiClient incoming = tcp_server.available();
    if (incoming) {
        int slot = -1;
        /* 优先复用已断开的槽位 */
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (!tcp_clients[i] || !tcp_clients[i].connected()) {
                slot = i;
                break;
            }
        }
        if (slot >= 0) {
            tcp_clients[slot] = incoming;
            tcp_clients[slot].setNoDelay(true);
            command_length=0; command_overflow=false;
            Serial2.print("\nSTOP\n");
            Serial.printf("[TCP] Client #%d connected: %s\r\n",
                          slot, tcp_clients[slot].remoteIP().toString().c_str());
            /* 向新客户端发送一行握手横幅 (上位机日志窗口可读, 不影响 $TELE 解析) */
            tcp_clients[slot].print("\r\n# Cubli-ESP WiFi Link OK (port 3333)\r\n");
        } else {
            /* 槽位已满: 拒绝新连接 */
            incoming.stop();
            Serial.println("[TCP] Too many clients, rejected.");
        }
    }
}

void setup()
{
    /* USB 调试串口 (仅本地排障用, 与数据链路无关) */
    Serial.begin(115200);
    delay(100);
    Serial.println("\r\n=== Cubli-ESP WiFi Bridge Booting ===");

    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    /* 1. 与 F407 的 UART 桥接通道 */
    Serial2.setRxBufferSize(2048);
    Serial2.begin(FOC_UART_BAUD, SERIAL_8N1, FOC_UART_RX_PIN, FOC_UART_TX_PIN);
    Serial.printf("[UART] Bridge to F407 ready: RX=IO%d, TX=IO%d, %d-8-N-1\r\n",
                  FOC_UART_RX_PIN, FOC_UART_TX_PIN, FOC_UART_BAUD);

    /* 2. WiFi: 先尝试 STA, 失败则回退 AP */
    Serial.printf("[WiFi] Trying STA: SSID=\"%s\"...\r\n", CUBLI_WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(CUBLI_WIFI_SSID, CUBLI_WIFI_PASS);

    uint32_t t0 = millis();
    bool sta_ok = false;
    while (millis() - t0 < 20000) {
        if (WiFi.status() == WL_CONNECTED) { sta_ok = true; break; }
        digitalWrite(STATUS_LED_PIN, (millis() / 200) % 2);  /* 快闪: STA 连接中 */
        delay(50);
    }

    if (sta_ok) {
        Serial.printf("[WiFi] STA connected! IP=%s RSSI=%d dBm\r\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("[WiFi] STA failed, switching to AP mode.");
        WiFi.disconnect();
        WiFi.mode(WIFI_AP);
        WiFi.softAP(CUBLI_AP_SSID, CUBLI_AP_PASS);
        Serial.printf("[WiFi] AP ready! SSID=\"%s\" PASS=\"%s\" IP=%s\r\n",
                      CUBLI_AP_SSID, CUBLI_AP_PASS,
                      WiFi.softAPIP().toString().c_str());
    }

    /* 3. TCP 服务器 */
    tcp_server.begin(CUBLI_TCP_PORT);
    tcp_server.setNoDelay(true);
    Serial.printf("[TCP] Server listening on port %d (up to %d clients)\r\n",
                  CUBLI_TCP_PORT, MAX_TCP_CLIENTS);

    IPAddress ip = (sta_ok ? WiFi.localIP() : WiFi.softAPIP());
    Serial.printf("[READY] UpperComputer: connect TCP to %s:%d\r\n",
                  ip.toString().c_str(), CUBLI_TCP_PORT);
}

void loop()
{
    /* 1. 维护 TCP 客户端 */
    accept_clients();
    cleanup_clients();

    bool any_client = false;
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcp_clients[i] && tcp_clients[i].connected()) { any_client = true; break; }
    }

    if(had_client && !any_client) {
        Serial2.print("\nSTOP\n");
        command_length=0; command_overflow=false;
    }
    had_client=any_client;

    /* 2. 上行透传: F407 UART -> 所有 TCP 客户端 */
    while (Serial2.available()) {
        uint8_t buf[256];
        size_t n = Serial2.read(buf, sizeof(buf));
        if (n == 0) break;
        stat_uart_to_tcp += n;
        if (any_client) {
            for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
                if (tcp_clients[i] && tcp_clients[i].connected()) {
                    if(tcp_clients[i].write(buf, n)!=n) {
                        tcp_clients[i].stop();
                        Serial2.print("\nSTOP\n");
                    }
                }
            }
        }
        /* 无客户端接入时数据直接丢弃 (F407 的 $TELE 周期流本身可丢帧) */
    }

    /* 3. 下行透传: 任意 TCP 客户端 -> F407 UART */
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (tcp_clients[i] && tcp_clients[i].connected() && tcp_clients[i].available()) {
            uint8_t buf[256];
            size_t n = tcp_clients[i].read(buf, sizeof(buf));
            if (n > 0) {
                stat_tcp_to_uart += n;
                for(size_t k=0;k<n;k++) {
                    char c=(char)buf[k];
                    if(c=='\n') {
                        if(!command_overflow && command_length) {
                            Serial2.write((const uint8_t*)command_line, command_length);
                            Serial2.write('\n');
                        } else if(command_overflow) Serial2.print("\nSTOP\n");
                        command_length=0; command_overflow=false;
                    } else if(c!='\r') {
                        if(c<32 || c>126 || command_length>=sizeof(command_line)-1) command_overflow=true;
                        else command_line[command_length++]=c;
                    }
                }
            }
        }
    }

    /* 4. 状态 LED: 有客户端=常亮; 无客户端=1Hz 慢闪 */
    uint32_t now = millis();
    if (any_client) {
        digitalWrite(STATUS_LED_PIN, HIGH);
    } else {
        if (now - led_last_toggle_ms >= 500) {
            led_last_toggle_ms = now;
            led_state = !led_state;
            digitalWrite(STATUS_LED_PIN, led_state);
        }
    }

    /* 5. 每 5s 打印一次透传统计 (USB 串口) */
    if (now - stat_last_print_ms >= 5000) {
        stat_last_print_ms = now;
        Serial.printf("[STAT] UART->TCP %u B, TCP->UART %u B, clients=%d, WiFi=%s\r\n",
                      (unsigned)stat_uart_to_tcp, (unsigned)stat_tcp_to_uart,
                      (any_client ? 1 : 0),
                      (WiFi.status() == WL_CONNECTED) ? "STA" : "AP");
    }
}
