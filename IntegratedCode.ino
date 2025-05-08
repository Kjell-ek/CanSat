#include <OneWire.h> 
#include <DallasTemperature.h>
#include <dht11.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Addresses
int ADXL345 = 0x1D; // The ADXL345 sensor I2C address
const int compassAddr = 0x0C; // I2C address for AK09915


#define DS18B20_PIN 8
#define DHT11PIN 4
#define PIR_PIN 3
#define GAS A0
const int trigPin = 9;
const int echoPin = 10;

// Defines variables
const float speedOfSound = 0.0343; // Speed of sound in cm/microseconds (assuming room temperature)
const int numMeasurements = 10; // Number of measurements to take for averaging
long duration;
float distance; 
float X_out, Y_out, Z_out;  // Accel Outputs
float magX, magY, magZ;  // Outputs from AK09915
int calibrationTime = 30;
const float declinationAngle = 9.49;

// Models
dht11 DHT11;

//Setup communication between oneWire and sensor
OneWire oneWire(DS18B20_PIN);

//Send reference to DallasTemp.
DallasTemperature sensors( &oneWire );

void setup(void)
{
  pinMode(PIR_PIN,INPUT); 
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  sensors.begin();          // Start sensor wiring
  Wire.begin();           
  setMeasurementMode(true);
  setContinuousMode();
  Serial.begin(9600);
 
  //Give the PIR sensor time for calibration
  Serial.print("Calibrating sensor ");
  for(int i = 0; i < calibrationTime; i++){
    Serial.print(".");
    delay(1000);
  }
  Serial.println(" done");
  Serial.println("SENSOR ACTIVE");
  delay(500);

}
void loop(void)
{
  // Ultrasonic
  float totalDistance = 0.0;
  unsigned long startTime;
  unsigned long endTime;

  // Take multiple measurements and average them
  startTime = millis(); // Record start time
  for (int i = 0; i < numMeasurements; i++) {
    totalDistance += getDistance();
    delay(50); // Delay between measurements to reduce interference
  }
  endTime = millis(); // Record end time

  distance = 1.0277*(totalDistance / numMeasurements) - 0.3245; // Calibrated Coefficients

  // Calculate the time taken for the distance measurements
  unsigned long elapsedTime = endTime - startTime;

  if(distance < 400){
    // Prints the distance and time on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm\t Time: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");
  }
  else{
    Serial.println("Out of range!");
  }
  // PIR

  // Sees if PIR recognize motion
  if(digitalRead(PIR_PIN) == HIGH){
    Serial.println("Motion detected!");
    //motion = HIGH;
  }
  
  // Sends you an acknowledgements if PIR doesn't recognize motion
  if(digitalRead(PIR_PIN) == LOW){
    Serial.println("Motion stopped!");
    //motion = LOW;
  }

  //Thermometer
  sensors.requestTemperatures();
  
  Serial.print("DS18B20 Temperature: ");
  Serial.print(sensors.getTempCByIndex(0)-0.9); // Calibrated constant
  Serial.println("°C ");

  // Temperature and Humidity sensor
  int chk = DHT11.read(DHT11PIN);

  Serial.print("Humidity (%): ");
  Serial.println((float)1.1759*DHT11.humidity-4.6776,2); // Calibration Coefficients

  Serial.print("DHT11 Temperature (°C): ");
  Serial.println((float)0.7435*DHT11.temperature-(-5.1098),2); // Calibration Coefficients

  // Accelerometer
  readAccelerometerData();

  // Compass & Accelerometer combiner

  if (dataReady()) {
    readMagnetometer(magX, magY, magZ);
    float headingMagnetic = calculateHeading(X_out,Y_out,Z_out,magX,magY,magZ);
    float headingTrue = headingMagnetic + declinationAngle;
    
    // Adjust true heading to be within the 0-360 degree range
    if (headingTrue < 0) headingTrue += 360;
    if (headingTrue >= 360) headingTrue -= 360;

    printSensorData();

    Serial.print("Heading (Magnetic North): ");
    Serial.println(headingMagnetic);

    Serial.print("Heading (True North): ");
    Serial.println(headingTrue);
    
    clearStatus(); // Ensure DRDY is cleared by reading the status register
  } else {
    Serial.println("Data not ready");
    setContinuousMode(); // Re-initialize the mode if data is not ready
  }

  // Gas Sensor
  Serial.print("Gas: ");
  Serial.println((int)analogRead(GAS));

  Serial.println();
  delay(1000);
}

float getDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Wait for the pulse to start
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  float distance = (duration * speedOfSound) / 2.0;

  return distance;
}

void readAccelerometerData() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true);
  
  X_out = (Wire.read() | Wire.read() << 8);
  X_out = (X_out / 256);
  Y_out = (Wire.read() | Wire.read() << 8);
  Y_out = Y_out / 256;
  Z_out = (Wire.read() | Wire.read() << 8);
  Z_out = (Z_out / 256);

  Serial.print("Xa= ");
  Serial.print(X_out);
  Serial.print("   Ya= ");
  Serial.print(Y_out);
  Serial.print("   Za= ");
  Serial.println(Z_out);
}

void setMeasurementMode(bool enable) {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D); // POWER_CTL register
  if (enable)
    Wire.write(0x08); // Bit D3 High for measuring enable
  else
    Wire.write(0x00); // Clear the measure bit to put the device into standby mode
  Wire.endTransmission();
}

void setContinuousMode() {
  Wire.beginTransmission(compassAddr);
  Wire.write(0x31); // Assuming CNTL2 register for setting mode
  Wire.write(0x08); // Continuous measurement mode at 100Hz
  Wire.endTransmission();
}

bool dataReady() {
  Wire.beginTransmission(compassAddr);
  Wire.write(0x10); // ST1 register to check data ready status
  Wire.endTransmission(false);
  Wire.requestFrom(compassAddr, 1,true);
  
  return (Wire.read() & 0x01);
}

void readMagnetometer(float &magX, float &magY, float &magZ) {
  Wire.beginTransmission(compassAddr);
  Wire.write(0x11); // Assuming HXL register start
  Wire.endTransmission(false);
  Wire.requestFrom(compassAddr, 6, true);

  if (Wire.available() == 6) {
    magX = (Wire.read() | (Wire.read() << 8)) * 0.15; // Adjust these calculations based on sensor sensitivity
    magY = (Wire.read() | (Wire.read() << 8)) * 0.15;
    magZ = (Wire.read() | (Wire.read() << 8)) * 0.15;
  }
}

void clearStatus() {
  Wire.beginTransmission(compassAddr);
  Wire.write(0x18); // ST2 register to clear any status flags
  Wire.endTransmission(false);
  Wire.requestFrom(compassAddr, 1);
  if (Wire.available()) {
    byte status = Wire.read(); // Read to clear the register
  }
}

float calculateHeading(float ax, float ay, float az, float mx, float my, float mz) {
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
  if (heading < 0) heading += 360;

  return heading;
}

void printSensorData() {
  Serial.print("Accelerometer (g): X=");
  Serial.print(X_out);
  Serial.print(", Y=");
  Serial.print(Y_out);
  Serial.print(", Z=");
  Serial.println(Z_out);
  
  Serial.print("Magnetometer (uT): X=");
  Serial.print(magX);
  Serial.print(", Y=");
  Serial.print(magY);
  Serial.print(", Z=");
  Serial.println(magZ);
}

void setOutputDataRate(byte rate) {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2C); // BW_RATE register
  Wire.write(rate); // Set data rate (Refer to the datasheet for possible values)
  Wire.endTransmission();
}