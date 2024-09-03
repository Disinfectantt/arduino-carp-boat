#include "esp32.h"

Esp32Controller::Esp32Controller()
    : radio(CE_PIN, CSN_PIN), server(80), ws("/ws"),
      RadioData{0, 0, 0.0f, 0.0f, false, false, 0.0f, 0.0f},
      gpsData{0.0f, 0.0f, 0.0f} {

#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif

  if (!LittleFS.begin(true)) {
#ifdef DEBUG_MODE
    Serial.println("An Error has occurred while mounting LittleFS");
#endif
    return;
  }

  sqlite3 *db;
  if (sqlite3_open(DB_PATH, &db) != SQLITE_OK) {
#ifdef DEBUG_MODE
    Serial.println("Failed to open database");
#endif
    return;
  }
  const char *sqlCreateTable =
      "CREATE TABLE IF NOT EXISTS points (id INTEGER PRIMARY KEY "
      "AUTOINCREMENT, name VARCHAR(100), lat REAL, lon REAL);";
  if (sqlite3_exec(db, sqlCreateTable, 0, 0, nullptr) != SQLITE_OK) {
    return;
  }
  sqlite3_close(db);

  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
#ifdef DEBUG_MODE
    Serial.println("Connecting to WiFi...");
#endif
  }
#ifdef DEBUG_MODE
  Serial.println("Connecting to WiFi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
#endif

  ws.onEvent(std::bind(&Esp32Controller::onWebSocketEvent, this,
                       std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4,
                       std::placeholders::_5, std::placeholders::_6));
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.begin();

  while (!radio.begin()) {
#ifdef DEBUG_MODE
    Serial.println("Radio doesn t work");
#endif
    delay(1000);
  }
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

  pinMode(JOY_L_Y_PIN, INPUT);
  pinMode(JOY_R_X_PIN, INPUT);

  buttonsTimer.start(20);
  receiveTimer.start(50);
}

void Esp32Controller::loop() {
  if (buttonsTimer.ready()) {
    getButtonsDataAndSend();
  }

  if (receiveTimer.ready() && radio.available()) {
    receiveAndSendGpsData();
  }
}

void Esp32Controller::onWebSocketEvent(AsyncWebSocket *server,
                                       AsyncWebSocketClient *client,
                                       AwsEventType type, void *arg,
                                       uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    sendAllPoints(client);
  } else if (type == WS_EVT_DATA) {
    JsonDocument doc;
    deserializeJson(doc, data);
    std::string action = doc["action"].as<std::string>();
    if (action == "savePoint") {
      savePoint(&doc);
    } else if (action == "deletePoint") {
      deletePoint(doc["id"]);
    } else if (action == "setHome") {
      setHome(&doc);
    } else if (action == "enableAutopilot") {
      RadioData.autopilotEnabled = doc["enabled"];
      RadioData.autopilotLat = doc["lat"];
      RadioData.autopilotLon = doc["lon"];
    }
  }
}

void Esp32Controller::receiveAndSendGpsData() {
  radio.startListening();
  if (radio.available()) {
    radio.read(&gpsData, sizeof(GPSData));
    String json = "{\"action\":\"updatePosition\",\"lat\":" +
                  String(gpsData.latitude, 6) +
                  ",\"lon\":" + String(gpsData.longitude, 6) +
                  ",\"course\":" + String(gpsData.course, 2) + "}";
    ws.textAll(json);
  }
  radio.stopListening();
}

void Esp32Controller::getButtonsDataAndSend() {
  int rawValueY = analogRead(JOY_L_Y_PIN);
  setDeadZone(6, &rawValueY);
  int rawValueX = analogRead(JOY_R_X_PIN);
  setDeadZone(6, &rawValueX);
  RadioData.y = map(rawValueY, 0, 4095, -255, 255);
  RadioData.x = map(rawValueX, 0, 4095, -255, 255);

  radio.write(&RadioData, sizeof(Data));
}

void Esp32Controller::setDeadZone(int n, int *d) {
  if (*d >= JOYSTICK_CENTER - n && *d <= JOYSTICK_CENTER + n) {
    *d = 0;
  }
}

void Esp32Controller::sendAllPoints(AsyncWebSocketClient *client) {
  OPEN_CONNECT_TO_DB;
  sqlite3_stmt *res;
  std::string sql = "SELECT * FROM points;";
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &res, nullptr) == SQLITE_OK) {
    JsonDocument pointDoc;
    String jsonString;
    while (sqlite3_step(res) == SQLITE_ROW) {
      pointDoc.clear();
      pointDoc["action"] = "addPoint";
      pointDoc["id"] = sqlite3_column_int(res, 0);
      pointDoc["name"] = (const char *)sqlite3_column_text(res, 1);
      pointDoc["lat"] = sqlite3_column_double(res, 2);
      pointDoc["lon"] = sqlite3_column_double(res, 3);
      jsonString.clear();
      serializeJson(pointDoc, jsonString);
      client->text(jsonString);
    }
    sqlite3_finalize(res);
  }
  sqlite3_close(db);
}

void Esp32Controller::savePoint(JsonDocument *doc) {
  OPEN_CONNECT_TO_DB;
  sqlite3_stmt *stmt;
  int currentCount = 0;
  const char *countSql = "SELECT COUNT(*) FROM points;";
  if (sqlite3_prepare_v2(db, countSql, -1, &stmt, nullptr) == SQLITE_OK) {
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      currentCount = sqlite3_column_int(stmt, 0);
    } else {
      return;
    }
    sqlite3_finalize(stmt);
  } else {
    return;
  }
  if (currentCount >= MAX_POINTS) {
    return;
  }
  std::string name = (*doc)["name"].as<std::string>();
  float lat = (*doc)["lat"];
  float lon = (*doc)["lon"];
  std::string sql = "INSERT INTO points (name, lat, lon) VALUES (?, ?, ?);";
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
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
  sqlite3_close(db);
}

void Esp32Controller::deletePoint(int id) {
  OPEN_CONNECT_TO_DB;
  sqlite3_stmt *stmt;
  std::string sql = "DELETE FROM points WHERE id = ?;";
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
    sqlite3_bind_int(stmt, 1, id);
    sqlite3_step(stmt);
    sqlite3_finalize(stmt);
  }
  sqlite3_close(db);
}

sqlite3 *Esp32Controller::openConnectToDb() {
  sqlite3 *db;
  if (sqlite3_open(DB_PATH, &db) != SQLITE_OK) {
#ifdef DEBUG_MODE
    Serial.println("Failed to open database");
#endif
    return nullptr;
  }
  return db;
}

void Esp32Controller::setHome(JsonDocument *doc) {
  float lat = (*doc)["lat"];
  float lon = (*doc)["lon"];
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
}
