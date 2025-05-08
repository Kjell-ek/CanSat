/** -- Dependencies --
Adafruit ADXL345 by Adafruit
ArduinoJson by Benoit Blanchon...
DallasTemperature by Miles Burton...
OneWire by Jim Studt,...
RF24 by TMRh20
U8g2 by oliver
https://github.com/adidax/dht11
*/

#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>

#include <OneWire.h> 
#include <DallasTemperature.h>
#include <dht11.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <AESLib.h>
#include <SoftwareReset.hpp>
#include "types.h"


// Addresses
const int ADXL345 = 0x1D; // The ADXL345 sensor I2C address
const int compassAddr = 0x0C; // I2C address for AK09915

#define CE_PIN 7 // Chip enable
#define CSN_PIN 4 // chip select not
#define DS18B20_PIN 8 // Temperature sensor
#define DHT11PIN 2
#define PIR_PIN 3
#define GAS A2
const int trigPin = 9; // ultrasonic activate
const int echoPin = 10; // ultrasonic receive

// Defines variables
const float speedOfSound = 0.0343; // Speed of sound in cm/microseconds (assuming room temperature)
const int numMeasurements = 1; // Number of measurements to take for averaging ultra sonic
const int calibrationTime = 10; // seconds of pir
const uint8_t address[][6] = { "1Node", "2Node" };
const bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
const int launchTime = 60; // seconds, time for the launch payload, think about calibration!

bool oor = false; // out of range
bool crashing = false; // activated with payload command from ground station
bool launching = true;
long now = 0; // timestamp n






// Models
RF24 radio(CE_PIN,CSN_PIN); 
dht11 DHT11;
//Setup communication between oneWire and sensor
OneWire oneWire(DS18B20_PIN);
//Send reference to DallasTemp.
DallasTemperature sensors( &oneWire );

ReceiveStruct received = {"First", 0};

void setupPins(){
  pinMode(PIR_PIN,INPUT); 
  // ultra sonic is not working on pin 9 and 10 with the antenna o.0
  //pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  //pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
}

void setupSensors(){
  sensors.begin(); // Start sensor wiring
  Wire.begin();
  acc_setMeasurementMode(true);
  mag_setContinuousMode();
}

void calibratePIR(){
  //Give the PIR sensor time for calibration
  //Serial.print("Calibrating PIR sensor ");
  for(int i = 0; i < calibrationTime; i++){
    //Serial.print(".");
    delay(1000);
  }
  //Serial.println(" done");
  //Serial.println("SENSOR ACTIVE");
}

void setupRadio() {
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    //Serial.println(F("radio hardware is not responding!!"));
    while (1) {}  // hold in infinite loop
  }
  delay(100);

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_MAX);  //TODO RF24_PA_MAX is default.

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

  //memcpy(payload.message, "Hello ", 6);  // set the payload message
  radio.stopListening();

  //printf_begin();
  //radio.printPrettyDetails();
}

// UltraSonic
float us_getDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Wait for the pulse to start
  long duration = pulseIn(echoPin, HIGH, 1004000);
  //Serial.println(duration);
  
  // Calculate the distance
  float distance = (duration * speedOfSound) / 2.0;

  return distance;
}

float us_measure(){
  // Ultrasonic
  float totalDistance = 0.0;
  for (int i = 0; i < numMeasurements; i++) {
    totalDistance += us_getDistance();
    delay(25); // Delay between measurements to reduce interference
  }
  // Calibrated Coefficients
  return 1.0277*(totalDistance / numMeasurements) - 0.3245;
}

// PIR
bool pir_recognizeMotion(){
  return digitalRead(PIR_PIN) == HIGH;
}

// Temperature
// temperature is shifted by 100 to send as an int
int temp_getCelcius(){
  sensors.requestTemperatures();
  //Serial.println(sensors.getTempCByIndex(0)-0.9);
  return (int)(((sensors.getTempCByIndex(0)-0.6)*100.23/98.8)*100);
}

// Temperature and Humidity
TempHum tah_read(){
  // TODO what says the return value?
  int chk = DHT11.read(DHT11PIN);
  // Calibration Coefficients
  float humidity = (float)1.1759*DHT11.humidity-4.6776;
  float celcius = (float)0.7435*DHT11.temperature-(-5.1098);
  return TempHum{humidity, celcius};
}

// Gas
int gas_read(){
  return (int)analogRead(GAS);
}

// Accelometer
void acc_readData(int &X_out, int &Y_out, int &Z_out) { // TODO dont use float, use int or two bytes
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true);
  
  X_out = (Wire.read() | Wire.read() << 8);
  Y_out = (Wire.read() | Wire.read() << 8);
  Z_out = (Wire.read() | Wire.read() << 8);
}

void acc_setMeasurementMode(bool enable) {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D); // POWER_CTL register
  if (enable)
    Wire.write(0x08); // Bit D3 High for measuring enable
  else
    Wire.write(0x00); // Clear the measure bit to put the device into standby mode
  Wire.endTransmission();
}

void acc_setOutputDataRate(byte rate) {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2C); // BW_RATE register
  Wire.write(rate); // Set data rate (Refer to the datasheet for possible values)
  Wire.endTransmission();
}

// Compass
void mag_setContinuousMode() {
  Wire.beginTransmission(compassAddr);
  Wire.write(0x31); // Assuming CNTL2 register for setting mode
  Wire.write(0x08); // Continuous measurement mode at 100Hz
  Wire.endTransmission();
}

bool mag_dataReady() {
  Wire.beginTransmission(compassAddr);
  Wire.write(0x10); // ST1 register to check data ready status
  Wire.endTransmission(false);
  Wire.requestFrom(compassAddr, 1,true);
  
  return (Wire.read() & 0x01);
}

void mag_readData(int &magX, int &magY, int &magZ) { // TODO dont use float, use int or two bytes
  Wire.beginTransmission(compassAddr);
  Wire.write(0x11); // Assuming HXL register start
  Wire.endTransmission(false);
  Wire.requestFrom(compassAddr, 6, true);

  if (Wire.available() == 6) {
    magX = (Wire.read() | (Wire.read() << 8)); 
    magY = (Wire.read() | (Wire.read() << 8));
    magZ = (Wire.read() | (Wire.read() << 8));
  }
}

void mag_clearStatus() {
  Wire.beginTransmission(compassAddr);
  Wire.write(0x18); // ST2 register to clear any status flags
  Wire.endTransmission(false);
  Wire.requestFrom(compassAddr, 1);
  if (Wire.available()) {
    byte status = Wire.read(); // Read to clear the register
  }
}

AccMagInt readAccMag(){
  AccMagInt data;
  acc_readData(data.acc_x, data.acc_y, data.acc_z);
  if (mag_dataReady()) {
    mag_readData(data.mag_x, data.mag_y, data.mag_z);
  } else {
    // TODO what is better?
    // filter bad data on gs
    data.mag_x = INVALID_DATA;
    data.mag_y = INVALID_DATA;
    data.mag_z = INVALID_DATA;
    mag_setContinuousMode(); // Re-initialize the mode if data is not ready
  }
  return data;
}

float seconds(long millis) {
  return millis / 1000;
}

void wait(long ms) {
  long t = ms - (millis()-now);
  if(t < 0)
    //Serial.println(F("Timing issue o.0"));
    return;
  else
    delay(t);
}

unsigned int buf_head = 0;
unsigned int buf_next = 0;
const int buf_size = 71;
// 0 1 2 3 4 5 6 index
// 0 0 0 0 0 0 0 buffer: 0 empty, 1 full
// head 0, next 0  pointer value for previous buffer state
// 1 0 0 0 0 0 0
// head 0, next 2
// 1 0 1 0 0 0 0
// head 0, next 4
// 1 0 1 0 1 0 0
// head 0, next 6
// 1 0 1 0 1 0 1
// head 0, next 8
// 1 1 1 0 1 0 1
// head 0, next 10
// 0 1 1 0 1 0 1
// head 2, next 10
// 0 1 0 0 1 0 1
// head 4, next 10
// 0 1 0 0 0 0 1
// head 6, next 10
// 0 1 0 0 0 0 0
// head 8, next 10
// 0 0 0 0 0 0 0
// head 10, next 10
// 0 0 0 1 0 0 0
// head 10, next 12

OutOfRangePayload buffer[buf_size];
void buf_push(OutOfRangePayload op){
  if(buf_next - buf_head >= 2 * buf_size) {
    // elements will be lost -> averaging
    buf_head += 2;
    int idx_after = (buf_next + 2) % buf_size; // average on the next entry
    OutOfRangePayload p_now = buf_get();
    OutOfRangePayload p_after = buffer[idx_after];
    
    buffer[idx_after].meta.time = (p_after.meta.time + p_now.meta.time) / 2;
    buffer[idx_after].celcius = (p_after.celcius + p_now.celcius) / 2;
    buffer[idx_after].gas = (p_after.gas + p_now.gas) / 2;
    buffer[idx_after].acc_x = (p_after.acc_x + p_now.acc_x) / 2;
    buffer[idx_after].acc_y = (p_after.acc_y + p_now.acc_y) / 2;
    buffer[idx_after].acc_z = (p_after.acc_z + p_now.acc_z) / 2;
    
  }
  buffer[buf_next % buf_size] = op;
  buf_next += 2;
}
OutOfRangePayload buf_get(){
  return buffer[buf_head % buf_size];
}
void buf_pop(){
  buf_head += 2;
}
bool buf_has(){
  return buf_head < buf_next;
  
}

void onAck(){
  if(received.message == C_CRASH) {
      crashing = true;
      launching = false;
  } else if(received.message == C_LAUNCH) {
      crashing = false;
      launching = true;
  } else {
    crashing = false;
    launching = false;  
  }
}

template <typename T>
bool send(T & payload, bool send_buf = true) {
  unsigned int attempts = 0;
  unsigned long ts = micros();                  // start the timer
  
  bool report;
  uint8_t enc_payload[32] = { 0 };
  memcpy(&enc_payload, &payload, sizeof(payload));
  aes128_enc_multiple(key, enc_payload, sizeof(enc_payload));
  do {
    report = radio.write(&enc_payload, sizeof(enc_payload));
    if (report) {
      uint8_t pipe;
      if (radio.available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
        radio.read(&received, sizeof(received));  // get incoming ACK payload
        /*
        Serial.print(F(" Recieved "));
        Serial.print(radio.getDynamicPayloadSize());  // print incoming payload size
        Serial.print(F(" bytes on pipe "));
        Serial.print(pipe);  // print pipe number that received the ACK
        Serial.print(F(" ,ack message: "));
        Serial.print(received.message);
        Serial.print(F(" ,ack counter: "));
        Serial.println(received.counter);
        */
        onAck();
        /*
        if(send_buf && buf_has()){
          OutOfRangePayload op = buf_get();
          if(send(op, false)){
            buf_pop();
          }
        }
        */
      } else {
        //Serial.println(F(" Invalid: an empty ACK packet"));  // empty ACK packet received
        report = false;
      }
    } else {
      //Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }
    attempts++;
  } while(!report && attempts < 10 && ((micros() -ts) < 300000)); // TODO
  oor = !report;
  return report;
}

/*
int cal_acc_x = 0;
int cal_acc_y = 0;
int cal_acc_z = 0;
void calibrateAcc() {
  //Give the Acc sensor time for calibration
  Serial.print("Calibrating Acc sensor ");
  for(int i = 0; i < calibrationTime; i++){
    int xx, yy, zz;
    acc_readData(xx, yy, zz);
    cal_acc_x += xx;
    cal_acc_y += yy;
    cal_acc_z += zz;
    delay(1000);
  }
  cal_acc_x /= calibrationTime;
  cal_acc_y /= calibrationTime;
  cal_acc_z /= calibrationTime;
  Serial.print(cal_acc_x);
  Serial.print(" ");
  Serial.print(cal_acc_y);
  Serial.print(" ");
  Serial.println(cal_acc_z);
  
  Serial.println(" done");
  // 5 258 19
  Serial.println("SENSOR ACTIVE");
}
*/
void setup() {
  //pinMode(SS, OUTPUT);
  //digitalWrite(SS, HIGH);
  //Serial.begin(115200); // used also on the gs for serialization
  //while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  //}
  setupRadio();
  setupSensors();
  setupPins();

  calibratePIR();
  //calibrateAcc();

  delay(500);

  // TODO Serial should be removed before launch
  /*
  Serial.println(sizeof(OrbitPayload));
  Serial.println(sizeof(LaunchPayload));
  Serial.println(sizeof(AccMagPayload));
  Serial.println(sizeof(CrashPayload));
  Serial.println(sizeof(OutOfRangePayload));
  */
  //Serial.println(F("CanSat started"));
}

void loop() {  
  now = millis();

/*
  // BUffer Code
  Serial.println(F("Out of Range"));
  Serial.print(buf_head);
  Serial.print(" ");
  Serial.print(buf_next);
  Serial.print(" ");
  Serial.println(buf_size);
  OutOfRangePayload opp = {{0,1},0,0,0,0,0};
  if(!send(opp))
    buf_push(opp);
  wait(2000);
  return;
*/
  //launching = millis() < launchTime;
  if(buf_has()){
    OutOfRangePayload op = buf_get();
    if(send(op, false)){
      buf_pop();
    }
  }
  if(!oor && !crashing && launching){
    // launch payload every 2s
    // accmag payload
    
    AccMagPayload amp;
    amp.meta = {T_ACCMAG, now};
    amp.data = readAccMag();
    

    LaunchPayload lp;
    lp.meta = {T_LAUNCH, now};
    lp.us_distance = us_measure(); // takes >500ms  
   
    wait(750);

    amp.dt = millis() - now;
    amp.data2 = readAccMag();
    
    send(amp);
  
    lp.gas = gas_read();
    lp.us_distance2 = us_measure(); // takes >500ms

    wait(1550);
    
    AccMagPayload amp2;
    long now2 = millis();
    amp2.meta = {T_ACCMAG, now2};
    amp2.data = readAccMag();

    lp.pir = pir_recognizeMotion();
    lp.tah = tah_read();

    send(lp);
 
    wait(1900);
    amp2.dt = millis() - now2;
    amp2.data2 = readAccMag();
    send(amp2);
 
    wait(2000);
  } else if(!oor && !crashing && !launching) {
    // orbit payload every 2s
    // accmag payload    
    AccMagPayload amp;
    amp.meta = {T_ACCMAG, now};
    amp.data = readAccMag();
    
    OrbitPayload op;
    op.meta = {T_ORBIT, now};
    op.celcius = temp_getCelcius();
    //Serial.println(op.celcius);
    op.tah = tah_read();
    op.gas = gas_read();
    send(op);

    wait(500);
    amp.dt = millis() - now;
    amp.data2 = readAccMag();
    send(amp);

    wait(1000);
    AccMagPayload amp2;
    long now2 = millis();
    amp2.meta = {T_ACCMAG, now2};
    amp2.data = readAccMag();
    wait(1500);
    amp2.dt = millis() - now2;
    amp2.data2 = readAccMag();
    send(amp2);
    wait(2000);
  } else if(!oor && crashing && !launching){
    // crashing payload
    // TODO release mechanism
    CrashPayload cp;
    cp.meta = {T_CRASH, now};
    acc_readData(cp.acc_x, cp.acc_y, cp.acc_z);
    send(cp);
    wait(180); // TODO frequenz
  } else if(oor){
    // out of range
    OutOfRangePayload op;
    op.meta = {T_OUTOFRANGE, now};
    op.celcius = temp_getCelcius();
    op.gas = gas_read();
    acc_readData(op.acc_x, op.acc_y, op.acc_z);
    if(!send(op))
      buf_push(op);
    wait(2000);
  } else {
    //Serial.println(F("Invalid state")); // TODO roger mode
    // go to orbit mode
    crashing = false;
    launching = false;  
    softwareReset::simple();
  }
}
