/** -- Dependencies --
Adafruit ADXL345 by Adafruit
ArduinoJson by Benoit Blanchon...
DallasTemperature by Miles Burton...
OneWire by Jim Studt,...
RF24 by TMRh20
U8g2 by oliver
https://github.com/adidax/dht11
https://github.com/DavyLandman/AESLib
*/

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <AESLib.h>
#include "types.h"

// set Chip-Enable (CE) and Chip-Select-Not (CSN) radio setup pins
#define CE_PIN 9
#define CSN_PIN 10

#define BTN_CRASH 4

const float declinationAngle = 9.49;

/*
uint8_t data[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                    0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
                    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                    0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
*/
// receive state
unsigned long last_receive_time = 0;
// trajectory integration
#define GRAVITY 9.81
unsigned long last_time = 0;
float vx = 0;
float vy = 0;
float vz = 0;
float x = 0;
float y = 0;
float z = 0;
float rad_north = 0;

// data for lcd
int gas = 0;
float temperature = 0;
float humidity = 0;

unsigned long btn_now = 0; // timestamp of button switch to avoid quick cycle through
unsigned long lcd_now = 0; // timestamp of lcd, cyclic
int lcd_state = -1; // what shoes the lcd, -1 for no state

StaticJsonDocument<512> outgoing; // TODO bigger?
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0); // LCD display

// create RF24 radio object using selected CE and CSN pins
RF24 radio(CE_PIN, CSN_PIN);

uint8_t address[][6] = { "1Node", "2Node" }; // TODO Check adresses
// 0 uses address[0] to transmit, 1 uses address[1] to transmit
constexpr bool radioNumber = 0;

uint8_t payload[32] = { 0 };
ReceiveStruct received = { C_LAUNCH, 0 };

void setupPins() {
  pinMode(BTN_CRASH, INPUT);
}

void setupRadio() {
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    while (1) {}  // hold in infinite loop
  }


  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_MAX);  // TODO RF24_PA_MAX is default. RF24_PA_LOW

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();  // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  radio.setChannel(RF_CHANNEL);

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

  // load the payload for the first received transmission on pipe 0
  radio.writeAckPayload(1, &receive, sizeof(receive));

  radio.startListening();  // put radio in RX mode
}

bool receive() {
  uint8_t pipe;
  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getDynamicPayloadSize();
    radio.read(&payload, sizeof(payload));
    aes128_dec_multiple(key, payload, sizeof(payload));
    received.counter++;
    while(!radio.writeAckPayload(1, &received, sizeof(received))) {
      radio.flush_tx();
    }
    // serialize receive time and receive counter
    last_receive_time = millis();
    outgoing.clear();
    outgoing["type"] = T_RECEIVED;
    outgoing["time"] = last_receive_time;
    outgoing["counter"] = received.counter;
    serializeJson(outgoing, Serial);
    Serial.print('\n');
    return true;
  }
  return false;
}

void serializeLaunchPayload(LaunchPayload p) {
  outgoing.clear();
  outgoing["type"] = p.meta.type;
  outgoing["time"] = p.meta.time;
  outgoing["tah_humidity"] = p.tah.humidity;
  outgoing["tah_celcius"] = p.tah.celcius;
  outgoing["us_distance"] = p.us_distance;
  outgoing["us_distance2"] = p.us_distance2;
  outgoing["gas"] = p.gas;
  outgoing["pir"] = p.pir;
  serializeJson(outgoing, Serial);
  Serial.print('\n');
  temperature = p.tah.celcius;
  humidity = p.tah.humidity;
  gas = p.gas;
}

void serializeOrbitPayload(OrbitPayload p) {
  outgoing.clear();
  outgoing["type"] = p.meta.type;
  outgoing["time"] = p.meta.time;
  outgoing["celcius"] = ((float)p.celcius)/100.;
  outgoing["tah_humidity"] = p.tah.humidity;
  outgoing["tah_celcius"] = p.tah.celcius;
  outgoing["gas"] = p.gas;
  serializeJson(outgoing, Serial);
  Serial.print('\n');
  temperature = ((float)p.celcius)/100.;
  humidity = p.tah.humidity;
  gas = p.gas;
}

void serializeAccMagPayload(AccMagPayload p) {
  outgoing.clear();
  outgoing["type"] = p.meta.type;
  
  AccMagFloat amf = adjustAccMacData(p.data);

  outgoing["time"] = p.meta.time;
  outgoing["acc_x"] = amf.acc_x;
  outgoing["acc_y"] = amf.acc_y;
  outgoing["acc_z"] = amf.acc_z;
  outgoing["mag_x"] = amf.mag_x;
  outgoing["mag_y"] = amf.mag_y;
  outgoing["mag_z"] = amf.mag_z;
  
  float headingMagnetic;
  float headingTrue;
  if(amf.mag_x == INVALID_DATA){
    headingMagnetic = INVALID_DATA;
    headingTrue = INVALID_DATA;
  } else {
    headingMagnetic = rad_to_deg(calculateHeading(amf.mag_x, amf.mag_y, amf.mag_z));
    headingTrue = headingMagnetic + declinationAngle;
    // Adjust true heading to be within the 0-360 degree range
    if (headingTrue < 0) headingTrue += 360;
    if (headingTrue >= 360) headingTrue -= 360;
  }
  calculate_trajectory(p.meta.time, amf.acc_x, amf.acc_y, amf.acc_z);
  outgoing["vx"] = vx;
  outgoing["vy"] = vy;
  outgoing["vz"] = vz;
  outgoing["x"] = x;
  outgoing["y"] = y;
  outgoing["z"] = z;

  outgoing["headingMagnetic"] = headingMagnetic;
  outgoing["headingTrue"] = headingTrue;



  AccMagFloat amf2 = adjustAccMacData(p.data2);
  
  outgoing["time2"] = p.meta.time + p.dt;
  outgoing["acc_x2"] = amf2.acc_x;
  outgoing["acc_y2"] = amf2.acc_y;
  outgoing["acc_z2"] = amf2.acc_z;
  outgoing["mag_x2"] = amf2.mag_x;
  outgoing["mag_y2"] = amf2.mag_y;
  outgoing["mag_z2"] = amf2.mag_z;

  float headingMagnetic2;
  float headingTrue2;
  if(amf2.mag_x == INVALID_DATA){
    headingMagnetic2 = INVALID_DATA;
    headingTrue2 = INVALID_DATA;
  } else {
    headingMagnetic2 = rad_to_deg(calculateHeading(amf2.mag_x, amf2.mag_y, amf2.mag_z));
    headingTrue2 = headingMagnetic2 + declinationAngle;
    // Adjust true heading to be within the 0-360 degree range
    if (headingTrue2 < 0) headingTrue2 += 360;
    if (headingTrue2 >= 360) headingTrue2 -= 360;
  }
  calculate_trajectory(p.meta.time + p.dt, amf2.acc_x, amf2.acc_y, amf2.acc_z);
  outgoing["vx2"] = vx;
  outgoing["vy2"] = vy;
  outgoing["vz2"] = vz;
  outgoing["x2"] = x;
  outgoing["y2"] = y;
  outgoing["z2"] = z;

  outgoing["headingMagnetic2"] = headingMagnetic2;
  outgoing["headingTrue2"] = headingTrue2;
  serializeJson(outgoing, Serial);
  Serial.print('\n');
}

void serializeOutOfRangePayload(OutOfRangePayload p) {
  outgoing.clear();
  outgoing["type"] = p.meta.type;
  outgoing["time"] = p.meta.time;

  float ax = ((float)p.acc_x) / 256;
  float ay = ((float)p.acc_y) / 256;
  float az = ((float)p.acc_z) / 256;
  outgoing["acc_x"] = ax;
  outgoing["acc_y"] = ay;
  outgoing["acc_z"] = az;
  calculate_trajectory(p.meta.time, ax, ay, az);
  outgoing["vx"] = vx;
  outgoing["vy"] = vy;
  outgoing["vz"] = vz;
  outgoing["x"] = x;
  outgoing["y"] = y;
  outgoing["z"] = z;

  outgoing["celcius"] = ((float)p.celcius)/100.;
  outgoing["gas"] = p.gas;
  serializeJson(outgoing, Serial);
  Serial.print('\n');
}

void serializeCrashPayload(CrashPayload p) {
  outgoing.clear();
  outgoing["type"] = p.meta.type;
  outgoing["time"] = p.meta.time;

  float ax = ((float)p.acc_x) / 256;
  float ay = ((float)p.acc_y) / 256;
  float az = ((float)p.acc_z) / 256;
  outgoing["acc_x"] = ax;
  outgoing["acc_y"] = ay;
  outgoing["acc_z"] = az;
  calculate_trajectory(p.meta.time, ax, ay, az);
  outgoing["vx"] = vx;
  outgoing["vy"] = vy;
  outgoing["vz"] = vz;
  outgoing["x"] = x;
  outgoing["y"] = y;
  outgoing["z"] = z;

  serializeJson(outgoing, Serial);
  Serial.print('\n');
}

AccMagFloat adjustAccMacData(AccMagInt & d){
  AccMagFloat r;
  r.acc_x = ((float)d.acc_x) / 256;
  r.acc_y = ((float)d.acc_y) / 256;
  r.acc_z = ((float)d.acc_z) / 256;

  if(d.mag_x == INVALID_DATA) {
    r.mag_x = INVALID_DATA;
    r.mag_y = INVALID_DATA;
    r.mag_z = INVALID_DATA;
  } else {
    // Adjust these calculations based on sensor sensitivity
    r.mag_x = ((float)d.mag_x) * 0.15;
    r.mag_y = ((float)d.mag_y) * 0.15;
    r.mag_z = ((float)d.mag_z) * 0.15;
  }
  return r;
}

// angle without correction in rad
// sets rad_north
float calculateHeading(float mx, float my, float mz) {
  /*
  // old params float ax, float ay, float az, 
  // Normalize accelerometer data
  float norm = sqrt(ax * ax + ay * ay + az * az);
  ax /= norm;
  ay /= norm;
  az /= norm;

  // Tilt compensation
  float pitch = asin(-ax);
  float roll = asin(ay / cos(pitch));

  // Adjust magnetometer values
  float mag_x = mx * cos(pitch) + mz * sin(pitch);
  float mag_y = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
  // Calculate heading
  float heading = atan2(-mag_y, mag_x) * 180 / PI;

  // Convert heading to 0 - 360 degrees
  while (heading < 0) heading += 360;
  return heading;
  */
  rad_north = atan2(-mz, mx);
  return rad_north;
}

float rad_to_deg(float rangle) {
  float heading = rangle * 180 / PI;
  while (heading < 0) heading += 360;
  return heading;
}

void calculate_trajectory(long new_time, float ax, float ay, float az) {
  if(last_time == 0) {
    last_time = new_time;
    return;
  }
  if(new_time < last_time) {
    return;
  }
  
  int aax = ax * 100;
  int aay = ay * 100;
  int aaz = az * 100;
  ax = (float)aax / 100.;
  ay = (float)aay / 100.;
  az = (float)aaz / 100.;
  /*
  outgoing["aax"] = ax;
  outgoing["aay"] = ay;
  outgoing["aaz"] = az;
  serializeJson(outgoing, Serial);
  Serial.print('\n');
*/
  float dt = ((float)(new_time - last_time))/1000; // dt in s
  // TODO cos/sin correction in acceleration or in velocity or in x,y,z?
  vx += ax * GRAVITY * cos(rad_north) * dt;
  vy += ay * GRAVITY * sin(rad_north) * dt;
  vz += az * GRAVITY * dt;
  x += vx * dt;
  y += vy * dt;
  z += vz * dt;
  last_time = new_time;
}

// m/s
float calculate_velocity() {
  return sqrt(vx*vx + vy*vy + vz*vz);
}

void lcd_text(String text1, String text2) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB14_tr);
    u8g2.drawStr(0,14,text1.c_str());
    u8g2.drawStr(0,32,text2.c_str());
  } while ( u8g2.nextPage() );
}

//MODERATE SMILEY
void lcd_moderate() {
  /*
  u8g2.firstPage();
  do {
    //EYES
    u8g2.drawCircle(42,9,5);
    u8g2.drawCircle(42,10,5);
    u8g2.drawCircle(88,9,5);
    u8g2.drawCircle(88,10,5);
    //MOUTH
    u8g2.drawLine(54,20,58,14);
    u8g2.drawLine(58,14,62,20);
    u8g2.drawLine(62,20,66,14);
    u8g2.drawLine(66,14,70,20);
    u8g2.drawLine(70,20,74,14);
    u8g2.drawLine(74,14,78,20);
  } while ( u8g2.nextPage() );
  */
  lcd_text("^ ^", " -");
}


//SAD SMILEY
void lcd_sad() {
  /*
  u8g2.firstPage();
  do {
    //EYES
    u8g2.drawCircle(42,9,5);
    u8g2.drawCircle(42,10,5);
    u8g2.drawCircle(88,9,5);
    u8g2.drawCircle(88,10,5);
    //MOUTH and EYEDROPS
    u8g2.drawCircle(66,24,9,3);
    u8g2.drawFilledEllipse(90,12,2,2);
    u8g2.drawFilledEllipse(90,19,1,2);
    u8g2.drawFilledEllipse(90,25,1,2);
  } while ( u8g2.nextPage() );
  */
  lcd_text(F("      x_x"), String((millis()-last_receive_time)/1000));
}

//HAPPY SMILEY
void lcd_happy() {
  /*
  u8g2.firstPage();
  do {
    //EYES
    u8g2.drawFilledEllipse(42,13,3,1);
    u8g2.drawEllipse(42,9,5,6);
    u8g2.drawEllipse(88,9,5,6);
    //MOUTH
    u8g2.drawCircle(66,18,9,12);
    u8g2.drawLine(57,18,75,18);
  } while ( u8g2.nextPage() );
  */
  lcd_text(F("      :)"), String(received.counter));
}

void lcd_switch() {
  lcd_state++;
  if(lcd_state == 0) {
    lcd_text(F("Speed m/s: "), String(calculate_velocity(), 2));
  } else if(lcd_state == 1) {
    lcd_text(F("Humidity: "), String(humidity, 2));
  } else if(lcd_state == 2) {
    lcd_text(F("CO2 ppm: "), String(gas));
  } else if(lcd_state == 3) {
    lcd_text(F("Temperature C: "), String(temperature, 2));
  } else if(lcd_state == 4) {
    // TODO face regarding state
    if(millis()-last_receive_time>2000)
      lcd_sad();
    else
      lcd_happy();
  } else if(lcd_state == 5) {
    if (received.message == C_LAUNCH) {
      lcd_text(F("Mode:"), "Launch");
    } else if (received.message == C_CRASH) {
      lcd_text(F("Mode:"), "Crash");
    } else {
      lcd_text(F("Mode:"), "Roger");
    }
  } else {
    lcd_state = -1;
  }
}

void button_switch() {
  if(digitalRead(BTN_CRASH) == HIGH && millis() - btn_now > 500) {
    btn_now = millis();
    if(received.message == C_LAUNCH){
      received.message = C_ROGER;
    } else if(received.message == C_ROGER){
      received.message = C_CRASH;
    } else {
      received.message = C_LAUNCH;
    }
  }
}

void setup() {
  Serial.begin(115200);  // used also on the gs for serialization
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  setupPins();
  setupRadio();
  u8g2.begin(); // setup LCD
  
  // TODO put crypto into receive and send, we dont encrypt ack, right?
  // void aes128_dec_multiple(const uint8_t* key, void* data, const uint16_t data_len);
  // void aes128_enc_multiple(const uint8_t* key, void* data, const uint16_t data_len);
  /*
  long now = millis();
  aes128_enc_multiple(key, data, sizeof(data));
  Serial.println(millis()-now);
  for(int i = 0; i < 32; i++)
    Serial.print(data[i], HEX);
  Serial.println();

  now = millis();
  aes128_dec_multiple(key, data, sizeof(data));
  Serial.println(millis()-now);

  for(int i = 0; i < 32; i++)
    Serial.print(data[i], HEX);
  Serial.println();
  */
  //delay(10000);
  Serial.println("Ground Station started");
}

void loop() {
  //radio.powerDown();
  //delay(150);
  //radio.powerUp();
  if(receive()){
    if(payload[0] == T_LAUNCH){
        LaunchPayload * lp = reinterpret_cast<LaunchPayload*>(payload);
        serializeLaunchPayload(*lp);
    } else if(payload[0] == T_ORBIT){
        OrbitPayload * op = reinterpret_cast<OrbitPayload*>(payload);
        serializeOrbitPayload(*op);
    } else if(payload[0] == T_ACCMAG){
        AccMagPayload * ap = reinterpret_cast<AccMagPayload*>(payload);
        serializeAccMagPayload(*ap);
    } else if(payload[0] == T_OUTOFRANGE){
        OutOfRangePayload * oop = reinterpret_cast<OutOfRangePayload*>(payload);
        serializeOutOfRangePayload(*oop);
    } else if(payload[0] == T_CRASH){
        CrashPayload * cp = reinterpret_cast<CrashPayload*>(payload);
        serializeCrashPayload(*cp);
    }
  }
  if(millis()-lcd_now>1500) {
    lcd_now = millis();
    lcd_switch();
  }
  button_switch();

  // TODO buf average and buffer
  // TODO encryption
  // TODO path calculation
}
