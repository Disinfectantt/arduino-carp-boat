#ifndef ESP_32_H_
#define ESP_32_H_

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <RF24.h>
#include <SPI.h>
#include <WiFi.h>
#include <data.h>
#include <nRF24L01.h>
#include <sqlite3.h>

#include <string>

#define MAX_POINTS 5000

#define SSID "ESP_32_AP"
#define PASSWORD "12345678"
#define HOSTNAME "gaypad"

#define TIMER_BUTTONS 10
#define SEND_GPS_TO_FRONTEND 600
#define TIMER_RECEIVE 100
#define WIFI_RECONNECT_TIMER 1000

#define RADIO_SWITCH_DELAY 5

// radio
#define CE_PIN 16
#define CSN_PIN 5

#define JOY_L_Y_PIN 34
#define JOY_R_X_PIN 35

#define JOYSTICK_CENTER 1900
#define JOYSTICK_DEAD_ZONE 50

#define OPEN_CONNECT_TO_DB         \
  sqlite3 *db = openConnectToDb(); \
  if (db == nullptr) return

const char DB_PATH[] = "/littlefs/points.db";

class Esp32Controller {
 private:
  RF24 radio;
  GPSData gpsData;
  Data RadioData;
  AsyncWebServer server;
  AsyncWebSocket ws;
  bool isAccess;
  bool isNetworkStarted;
  SemaphoreHandle_t xMutex;

 private:
  void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                        AwsEventType type, void *arg, uint8_t *data,
                        size_t len);
  void receiveGpsData();
  void getButtonsDataAndSend();
  void setDeadZone(int y, int x);
  void savePoint(JsonDocument *doc);
  void deletePoint(int id);
  static sqlite3 *openConnectToDb();
  void setHome(JsonDocument *doc);
  void readWiFiCredentials(String &ssid, String &password);
  void saveWiFiCredentials(String &ssid, String &password);
  void startAccess();
  void initServer();
  void initWifi();
  bool connectWifiAsync(String &ssid, String &password);
  void attemptConnectWifi();
  bool initRadio();
  bool initDb();
  void sendGpsToFront();

  static void buttonsTask(void *param);
  static void receiveTask(void *param);
  static void wifiReconnectTask(void *param);
  static void wifiConnectTask(void *param);
  static void sendGpsToFrontTask(void *param);

  static void sendAllPoints(void *client);
  static void processWebsocket(void *param);

  struct webSocketWrapper {
    uint8_t *data;
    Esp32Controller *esp;
  };

  struct wifiWrapper {
    Esp32Controller *esp;
    String &ssid;
    String &pass;
  };

 public:
  Esp32Controller();
  void begin();
};

#endif