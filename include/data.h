#ifndef DATA_H_
#define DATA_H_

const byte address[6] = "Node1";

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

#endif