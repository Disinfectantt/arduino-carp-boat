#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <sqlite3.h>
#include <string>

const char ssid[] = "";
const char password[] = "";

const char DB_PATH[] = "/littlefs/points.db";

// radio
#define CE_PIN 16
#define CSN_PIN 5

#define JOY_L_Y_PIN 34
#define JOY_R_X_PIN 35

#define JOYSTICK_CENTER 1900

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001";

struct GPSData {
  float latitude;
  float longitude;
  float course;
};

struct Data {
  int16_t y;
  int16_t x;
  float autopilotLat;
  float autopilotLon;
  bool autopilotEnabled;
  bool isHome;
  float homeLat;
  float homeLon;
};

GPSData gpsData;
Data RadioData;

sqlite3 *db;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setDeadZone(int n, int *d) {
  if (*d >= JOYSTICK_CENTER - n && *d <= JOYSTICK_CENTER + n) {
    *d = 0;
  }
}

void sendAllPoints(AsyncWebSocketClient *client) {
  sqlite3_stmt *res;
  const char *tail;
  JsonDocument doc;
  doc["action"] = "pointsList";
  JsonArray points = doc.createNestedArray("points");

  std::string sql = "SELECT * FROM points;";
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &res, &tail) == SQLITE_OK) {
    while (sqlite3_step(res) == SQLITE_ROW) {
      JsonObject pointObj = points.createNestedObject();
      pointObj["id"] = sqlite3_column_int(res, 0);
      pointObj["name"] = (const char *)sqlite3_column_text(res, 1);
      pointObj["lat"] = sqlite3_column_double(res, 2);
      pointObj["lon"] = sqlite3_column_double(res, 3);
    }
    sqlite3_finalize(res);
  }

  String jsonString;
  serializeJson(doc, jsonString);
  client->text(jsonString);
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    sendAllPoints(client);
  } else if (type == WS_EVT_DATA) {
    if (sqlite3_open(DB_PATH, &db) != SQLITE_OK) {
      // Serial.println("Failed to open database");
      return;
    }
    JsonDocument doc;
    deserializeJson(doc, data);
    std::string action = doc["action"].as<std::string>();
    sqlite3_stmt *stmt;
    if (action == "savePoint") {
      std::string name = doc["name"].as<std::string>();
      float lat = doc["lat"];
      float lon = doc["lon"];
      std::string sql = "INSERT INTO points (name, lat, lon) VALUES (?, ?, ?);";
      if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, NULL) == SQLITE_OK) {
        sqlite3_bind_text(stmt, 1, name.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_double(stmt, 2, lat);
        sqlite3_bind_double(stmt, 3, lon);
        if (sqlite3_step(stmt) == SQLITE_DONE) {
          int newId = sqlite3_last_insert_rowid(db);
          JsonDocument responseDoc;
          responseDoc["action"] = "addPoint";
          responseDoc["id"] = newId;
          responseDoc["name"] = name;
          responseDoc["lat"] = lat;
          responseDoc["lon"] = lon;
          String jsonResponse;
          serializeJson(responseDoc, jsonResponse);
          ws.textAll(jsonResponse);
        }
        sqlite3_finalize(stmt);
      }
    } else if (action == "deletePoint") {
      int id = doc["id"];
      std::string sql = "DELETE FROM points WHERE id = ?;";
      if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, NULL) == SQLITE_OK) {
        sqlite3_bind_int(stmt, 1, id);
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
      }
    } else if (action == "setHome") {
      float lat = doc["lat"];
      float lon = doc["lon"];
      JsonDocument responseDoc;
      responseDoc["action"] = "setHome";
      responseDoc["lat"] = lat;
      responseDoc["lon"] = lon;
      String jsonResponse;
      serializeJson(responseDoc, jsonResponse);
      ws.textAll(jsonResponse);
      RadioData.isHome = true;
      RadioData.homeLat = lat;
      RadioData.homeLon = lon;
    } else if (action == "enableAutopilot") {
      RadioData.autopilotEnabled = doc["enabled"];
      RadioData.autopilotLat = doc["lat"];
      RadioData.autopilotLon = doc["lon"];
    }
    sqlite3_close(db);
  }
}

void setup() {
  // Serial.begin(115200);

  if (!LittleFS.begin(true)) {
    // Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  if (sqlite3_open(DB_PATH, &db) != SQLITE_OK) {
    // Serial.println("Failed to open database");
    return;
  }

  const char *sqlCreateTable = "CREATE TABLE IF NOT EXISTS points (id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT, lat REAL, lon REAL);";
  char *errMsg;
  if (sqlite3_exec(db, sqlCreateTable, 0, 0, &errMsg) != SQLITE_OK) {
    sqlite3_free(errMsg);
  }

  sqlite3_close(db);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    // Serial.println("Connecting to WiFi...");
  }
  // Serial.println("Connecting to WiFi");
  // Serial.print("IP: ");
  // Serial.println(WiFi.localIP());

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.begin();

  while (!radio.begin()) {
    // Serial.println("Radio doesn t work");
    delay(1000);
  }
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

  pinMode(JOY_L_Y_PIN, INPUT);
  pinMode(JOY_R_X_PIN, INPUT);

  RadioData.autopilotEnabled = false;
  RadioData.isHome = false;
}

void loop() {
  int rawValueY = analogRead(JOY_L_Y_PIN);
  setDeadZone(6, &rawValueY);
  int rawValueX = analogRead(JOY_R_X_PIN);
  setDeadZone(6, &rawValueX);
  RadioData.y = map(rawValueY, 0, 4095, -255, 255);
  RadioData.x = map(rawValueX, 0, 4095, -255, 255);

  radio.write(&RadioData, sizeof(Data));

  radio.startListening();
  if (radio.available()) {
    radio.read(&gpsData, sizeof(GPSData));

    String json = "{\"action\":\"updatePosition\",\"lat\":" + String(gpsData.latitude, 6) + ",\"lon\":" + String(gpsData.longitude, 6) + ",\"course\":" + String(gpsData.course, 2) + "}";
    ws.textAll(json);
  }
  radio.stopListening();

  delay(50);
}
