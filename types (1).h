#define T_LAUNCH 0
#define T_ORBIT 1
#define T_ACCMAG 2
#define T_OUTOFRANGE 3
#define T_CRASH 4
#define T_RECEIVED 5 // not a payload type, only used on gs

// ack messages
#define C_ROGER 'R'
#define C_LAUNCH 'L'
#define C_CRASH 'C'

#define RF_CHANNEL 67

#define INVALID_DATA -32000

// Crypto
uint8_t key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

// TODO __attribute__ ((packed)); ? -> 8bit alignment makes no problem
struct Meta { // 5B
  uint8_t type; // type of the payload
  unsigned long time; // timestamp of measurement, TODO do we need long(32bit)?
};
struct TempHum { // 8B
  float humidity;
  float celcius;
};
struct AccMagInt { // 12B
  int acc_x;
  int acc_y;
  int acc_z;
  int mag_x;
  int mag_y;
  int mag_z;
};
struct AccMagFloat { // 24B
  float acc_x;
  float acc_y;
  float acc_z;
  float mag_x;
  float mag_y;
  float mag_z;
};
struct LaunchPayload { // 24B
  Meta meta;
  TempHum tah;
  float us_distance;
  float us_distance2;
  int gas;
  bool pir;
};
struct OrbitPayload { // 17B
  Meta meta;
  int celcius; // divide by 100 to float on gs
  TempHum tah;
  int gas;
};
struct AccMagPayload { // 31B
  Meta meta;
  AccMagInt data;
  unsigned int dt; // time diff of seconds measurement
  AccMagInt data2;
};
struct OutOfRangePayload { // 15B
  Meta meta;
  int acc_x;
  int acc_y;
  int acc_z;
  int celcius; // divide by 100 to float on gs
  int gas;
};
struct CrashPayload { // 11B
  Meta meta;
  int acc_x;
  int acc_y;
  int acc_z;
};

struct ReceiveStruct {
  uint8_t message;
  unsigned int counter;
};