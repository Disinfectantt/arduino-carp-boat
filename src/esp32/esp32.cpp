#include "esp32.h"

Esp32Controller::Esp32Controller()
    : radio(CE_PIN, CSN_PIN),
      server(80),
      ws("/ws"),
      RadioData{0, 0, 0.0f, 0.0f, false, false, 0.0f, 0.0f},
      gpsData{0.0f, 0.0f, 0},
      isAccess(false),
      isNetworkStarted(false),
      xMutex(NULL) {}

void Esp32Controller::begin() {
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif

  pinMode(JOY_L_Y_PIN, INPUT);
  pinMode(JOY_R_X_PIN, INPUT);

  xMutex = xSemaphoreCreateMutex();

  if (initRadio()) {
    xTaskCreate(buttonsTask, "Buttons", 2056, this, 5, NULL);
  }

  if (!LittleFS.begin()) {
#ifdef DEBUG_MODE
    Serial.println("An Error has occurred while mounting LittleFS");
#endif
    return;
  }

  if (!initDb()) {
    return;
  }

  initWifi();

  xTaskCreate(sendGpsToFrontTask, "front gps", 4056, this, 3, NULL);
  xTaskCreate(receiveTask, "Receive", 2056, this, 4, NULL);
  xTaskCreate(wifiReconnectTask, "WifiReconnect", 2056, this, 5, NULL);
}

void Esp32Controller::onWebSocketEvent(AsyncWebSocket *server,
                                       AsyncWebSocketClient *client,
                                       AwsEventType type, void *arg,
                                       uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    xTaskCreate(sendAllPoints, "All points", 5000, client, 3, NULL);
  } else if (type == WS_EVT_DATA) {
    webSocketWrapper *w = (webSocketWrapper *)malloc(sizeof(webSocketWrapper));
    w->data = data;
    w->esp = this;
    if (w != NULL) {
      xTaskCreate(processWebsocket, "processWebsocket", 5400, w, 2, NULL);
    }
  }
}

void Esp32Controller::processWebsocket(void *param) {
  webSocketWrapper *w = (webSocketWrapper *)param;
  JsonDocument doc;
  deserializeJson(doc, w->data);
  std::string action = doc["action"].as<std::string>();
  if (action == "savePoint") {
    w->esp->savePoint(&doc);
  } else if (action == "deletePoint") {
    w->esp->deletePoint(doc["id"]);
  } else if (action == "setHome") {
    w->esp->setHome(&doc);
  } else if (action == "enableAutopilot") {
    w->esp->RadioData.autopilotEnabled = doc["enabled"];
    w->esp->RadioData.autopilotLat = doc["lat"];
    w->esp->RadioData.autopilotLon = doc["lon"];
  }
  free(w);
#ifdef DEBUG_MODE
  Serial.printf("process websocket stack: %u\n",
                uxTaskGetStackHighWaterMark(NULL));
#endif
  vTaskDelete(NULL);
}

void Esp32Controller::sendGpsToFrontTask(void *param) {
  Esp32Controller *c = (Esp32Controller *)param;
  while (1) {
    c->sendGpsToFront();
    vTaskDelay(pdMS_TO_TICKS(SEND_GPS_TO_FRONTEND));
  }
}

void Esp32Controller::buttonsTask(void *param) {
  Esp32Controller *c = (Esp32Controller *)param;
  while (1) {
    if (xSemaphoreTake(c->xMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      c->getButtonsDataAndSend();
      xSemaphoreGive(c->xMutex);
    }
    // Serial.printf("buttons stack: %u\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(pdMS_TO_TICKS(TIMER_BUTTONS));
  }
}

void Esp32Controller::receiveTask(void *param) {
  Esp32Controller *c = (Esp32Controller *)param;
  while (1) {
    if (xSemaphoreTake(c->xMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      c->receiveGpsData();
      xSemaphoreGive(c->xMutex);
    }
    // Serial.printf("receive stack: %u\n", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(pdMS_TO_TICKS(TIMER_RECEIVE));
  }
}

void Esp32Controller::wifiReconnectTask(void *param) {
  Esp32Controller *c = (Esp32Controller *)param;
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
    // Serial.printf("wifi reconnect stack: %u\n",
    // uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_TIMER));
  }
}

bool Esp32Controller::initRadio() {
  if (!radio.begin()) {
#ifdef DEBUG_MODE
    Serial.println("Radio doesn t work");
#endif
    return false;
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.setRetries(15, 15);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setAddressWidth(5);

  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(0, address[1]);
  radio.startListening();
  return true;
}

bool Esp32Controller::initDb() {
  sqlite3 *db;
  if (sqlite3_open(DB_PATH, &db) != SQLITE_OK) {
#ifdef DEBUG_MODE
    Serial.println("Failed to open database");
#endif
    return false;
  }
  const char *sqlCreateTable =
      "CREATE TABLE IF NOT EXISTS points (id INTEGER PRIMARY KEY "
      "AUTOINCREMENT, name VARCHAR(100), lat REAL, lon REAL);";
  if (sqlite3_exec(db, sqlCreateTable, 0, 0, nullptr) != SQLITE_OK) {
#ifdef DEBUG_MODE
    Serial.println("Failed to create table");
#endif
    return false;
  }
  sqlite3_close(db);
  return true;
}

void Esp32Controller::initServer() {
  ws.onEvent(std::bind(&Esp32Controller::onWebSocketEvent, this,
                       std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4,
                       std::placeholders::_5, std::placeholders::_6));
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [&](AsyncWebServerRequest *request) {
    if (isAccess) {
      request->send(LittleFS, "/wifi.html", "text/html");
    } else {
      request->send(LittleFS, "/index.html", "text/html");
    }
  });
  server.on("/manifest.json", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/manifest.json", "application/json");
  });
  server.on("/service-worker.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/service-worker.js", "text/javascript");
  });
  server.on("/wifi_connect", HTTP_POST, [&](AsyncWebServerRequest *request) {
    if (isAccess) {
      String ssid = request->getParam("ssid", true)->value();
      String pass = request->getParam("password", true)->value();
      saveWiFiCredentials(ssid, pass);
      initWifi();
    }
  });

  server.begin();
}

void Esp32Controller::startAccess() {
  isAccess = true;
  WiFi.softAP(SSID, PASSWORD);
}

void Esp32Controller::initWifi() {
  xTaskCreate(wifiConnectTask, "Init Wifi", 4096, this, 5, NULL);
}

void Esp32Controller::wifiConnectTask(void *param) {
  Esp32Controller *e = (Esp32Controller *)param;
  e->attemptConnectWifi();
  vTaskDelete(NULL);
}

void Esp32Controller::attemptConnectWifi() {
  String ssid, password;
  readWiFiCredentials(ssid, password);
  if (ssid.length() == 0 && !isAccess) {
    startAccess();
  } else {
    if (!connectWifiAsync(ssid, password)) {
      startAccess();
    }
  }
  if (!isNetworkStarted) {
    isNetworkStarted = true;
    initServer();
    if (!MDNS.begin(HOSTNAME)) {
#ifdef DEBUG_MODE
      Serial.println("MDNS FAILED");
#endif
    }
  }
}

bool Esp32Controller::connectWifiAsync(String &ssid, String &password) {
  int attempts = 0;
  if (isAccess) {
    WiFi.softAPdisconnect(false);
  }
  if (password.length() == 0) {
    WiFi.begin(ssid);
  } else {
    WiFi.begin(ssid, password);
  }
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    vTaskDelay(pdMS_TO_TICKS(10000));
    ++attempts;
#ifdef DEBUG_MODE
    Serial.println("Connecting to WiFi...");
#endif
  }
  if (WiFi.status() == WL_CONNECTED) {
    isAccess = false;
    return true;
  }
  return false;
}

void Esp32Controller::readWiFiCredentials(String &ssid, String &password) {
  File file = LittleFS.open("/wifi.txt", "r");
  if (!file) {
#ifdef DEBUG_MODE
    Serial.println("Failed to open wifi file for reading");
#endif
    return;
  }
  ssid = file.readStringUntil('\n');
  password = file.readStringUntil('\n');

  ssid.trim();
  password.trim();

  file.close();
}

void Esp32Controller::saveWiFiCredentials(String &ssid, String &password) {
  File file = LittleFS.open("/wifi.txt", "w");
  if (!file) {
#ifdef DEBUG_MODE
    Serial.println("Failed to open wifi file for writing");
#endif
    return;
  }
  file.println(ssid);
  file.println(password);
  file.close();
}

void Esp32Controller::receiveGpsData() {
  if (radio.available()) {
    radio.read(&gpsData, sizeof(GPSData));
  }
}

void Esp32Controller::getButtonsDataAndSend() {
  static float xFiltered = 0.0f;
  static float yFiltered = 0.0f;
  static const float alpha = 0.3f;  // Коэффициент сглаживания (0.1-0.5)

  int rawX = analogRead(JOY_R_X_PIN);
  int rawY = analogRead(JOY_L_Y_PIN);

  // EMA-фильтрация
  xFiltered = alpha * rawX + (1 - alpha) * xFiltered;
  yFiltered = alpha * rawY + (1 - alpha) * yFiltered;

  RadioData.y = (int)getDeadZoneAxis(yFiltered);
  RadioData.x = (int)getDeadZoneAxis(xFiltered);

  // Serial.printf("X: %d\n Y:%d\n", RadioData.x, RadioData.y);

  radio.stopListening();
  radio.write(&RadioData, sizeof(Data));
  radio.startListening();
}

float Esp32Controller::getDeadZoneAxis(float value) {
  if (value >= JOYSTICK_CENTER - JOYSTICK_DEAD_ZONE &&
      value <= JOYSTICK_CENTER + JOYSTICK_DEAD_ZONE) {
    return 0;
  } else {
    return constrain(mapFloat(value, 0, 4095, -255, 255), -255, 255);
  }
}

void Esp32Controller::sendAllPoints(void *client) {
  sqlite3 *db = openConnectToDb();
  if (db == nullptr) {
    vTaskDelete(NULL);
    return;
  }
  sqlite3_stmt *res;
  std::string sql = "SELECT * FROM points;";
  if (sqlite3_prepare_v2(db, sql.c_str(), -1, &res, nullptr) == SQLITE_OK) {
    JsonDocument pointDoc;
    String jsonString;
    while (sqlite3_step(res) == SQLITE_ROW) {
      pointDoc.clear();
      pointDoc["action"] = "addPoint";
      pointDoc["id"] = sqlite3_column_int(res, 0);
      pointDoc["name"] = (const unsigned char *)sqlite3_column_text(res, 1);
      pointDoc["lat"] = sqlite3_column_double(res, 2);
      pointDoc["lon"] = sqlite3_column_double(res, 3);
      jsonString.clear();
      serializeJson(pointDoc, jsonString);
      AsyncWebSocketClient *c = (AsyncWebSocketClient *)client;
      c->text(jsonString);
    }
    sqlite3_finalize(res);
  }
  sqlite3_close(db);
#ifdef DEBUG_MODE
  Serial.printf("Send all points task stack: %u\n",
                uxTaskGetStackHighWaterMark(NULL));
#endif
  vTaskDelete(NULL);
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

void Esp32Controller::sendGpsToFront() {
  static GPSData previousGpsData = {0.0f, 0.0f, 0};

  if (gpsData != previousGpsData) {
    JsonDocument responseDoc;
    responseDoc["action"] = "updatePosition";
    responseDoc["lat"] = String(gpsData.latitude, 6);
    responseDoc["lon"] = String(gpsData.longitude, 6);
    responseDoc["course"] = String(gpsData.course);
    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);
    ws.textAll(jsonResponse);

    previousGpsData = gpsData;
  }
}

float Esp32Controller::mapFloat(float value, float fromLow, float fromHigh,
                                float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
