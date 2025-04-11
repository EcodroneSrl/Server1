struct __attribute__((packed)) dummy {
  int32_t lat;
  int32_t lon;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint16_t signal;
} dummyData;

struct __attribute__((packed)) dummy_risp {
  int32_t rifLatTrue;
  int32_t rifLonTrue;
  int32_t rifLatMission;
  int32_t rifLonMission;
  int32_t gpsLat;
  int32_t gpsLon;
} dummy_risp_data;