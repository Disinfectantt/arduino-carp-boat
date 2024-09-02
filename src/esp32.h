#ifndef ESP_32_H_
#define ESP_32_H_

#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
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

#define SSID "OnePlus 9R"
#define PASSWORD "qwerty11"

const char DB_PATH[] = "/littlefs/points.db";

// radio
#define CE_PIN 16
#define CSN_PIN 5

#define JOY_L_Y_PIN 34
#define JOY_R_X_PIN 35

#define JOYSTICK_CENTER 1900

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001";

GPSData gpsData;
Data RadioData;

sqlite3 *db;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Timer buttonsTimer, receiveTimer;

#endif