#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_u-blox_GNSS_v3.h> 

// pin assignments
int sda_mpu = 35;
int scl_mpu = 36;
int sda_gps = 17;
int scl_gps = 16;

// declare vars
int freq = 100000;
double lat, lon, alt;
sensors_event_t a, g, temp;

// Create instance
Adafruit_MPU6050 mpu;
SFE_UBLOX_GNSS myGNSS; 

TwoWire I2CBusMPU = TwoWire(0); // First I2C bus
TwoWire I2CBusGPS = TwoWire(1); // Second I2C bus


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  I2CBusMPU.begin(sda_mpu, scl_mpu, freq);
  I2CBusGPS.begin(sda_gps, scl_gps, freq);
  if (!mpu.begin(0x68, &I2CBusMPU)) {
    Serial.println("Failed to find MPU6050 chip on I2CBusIMU!");
    while (1) yield();
  }

  if (myGNSS.begin(I2CBusGPS) == false) {
    Serial.println("GNSS not detected. Check wiring.");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.getEvent(&a, &g, &temp);
  lat = myGNSS.getLatitude();
  lon = myGNSS.getLongitude();
  alt = myGNSS.getAltitude();
  Serial.print("Accel X: "); Serial.print(a.acceleration.x);
  Serial.print(", Y: "); Serial.print(a.acceleration.y);
  Serial.print(", Z: "); Serial.print(a.acceleration.z);
  Serial.print(" | Gyro X: "); Serial.print(g.gyro.x);
  Serial.print(", Y: "); Serial.print(g.gyro.y);
  Serial.print(", Z: "); Serial.println(g.gyro.z);
  Serial.print("Lat: "); Serial.print(lat);
  Serial.print("  Lon: "); Serial.print(lon);
  Serial.print("  Alt: "); Serial.println(alt);
  delay(500);
}