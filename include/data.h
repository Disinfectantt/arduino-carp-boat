#ifndef DATA_H_
#define DATA_H_

const byte address[][6] = {"Node1", "Node2"};

#pragma pack(1)
struct GPSData {
  float latitude;
  float longitude;
  float course;
};
#pragma pack()

#pragma pack(1)
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
#pragma pack()

#endif