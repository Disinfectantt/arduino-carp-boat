#ifndef ESP_32_H_
#define ESP_32_H_

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <RF24.h>
#include <SPI.h>
#include <Timer.h>
#include <WiFi.h>
#include <data.h>
#include <nRF24L01.h>
#include <sqlite3.h>

#include <string>

#define MAX_POINTS 10000

#define SSID "ESP_32"
#define PASSWORD "admin"
#define HOSTNAME "transmitter"

#define TIMER_BUTTONS 10
#define TIMER_RECEIVE 20
#define WIFI_RECONNECT_TIMER 1000

// radio
#define CE_PIN 16
#define CSN_PIN 5

#define JOY_L_Y_PIN 34
#define JOY_R_X_PIN 35

#define JOYSTICK_CENTER 1900

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
  Timer buttonsTimer, receiveTimer, wifiReconnectTimer;
  bool isAccess;

 private:
  void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                        AwsEventType type, void *arg, uint8_t *data,
                        size_t len);
  void receiveAndSendGpsData();
  void getButtonsDataAndSend();
  void setDeadZone(int n, int *d);
  void sendAllPoints(AsyncWebSocketClient *client);
  void savePoint(JsonDocument *doc);
  void deletePoint(int id);
  sqlite3 *openConnectToDb();
  void setHome(JsonDocument *doc);
  void readWiFiCredentials(String &ssid, String &password);
  void saveWiFiCredentials(String &ssid, String &password);
  void startAccess();
  void handleRoot(AsyncWebServerRequest *request);
  void handleNewWifi(AsyncWebServerRequest *request);
  void initServer();
  void initWifi();
  bool connectWifi(String &ssid, String &password);

 public:
  Esp32Controller();
  void loop();
};

#endif