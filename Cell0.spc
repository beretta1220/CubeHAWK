#include <Wire.h>
#include <BMI088.h>
#include <RM3100.h>
#include <Madgwick.h>
#include <Adafruit_TSL2591.h>
#include <math.h>

#define IMU_ADDR 0x19
#define MAG_ADDR 0x20
#define TSL_ADDR 0x29

BMI088 imu;
RM3100 mag;
Madgwick filter;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

void setup() {
  Wire.begin();
  imu.begin(IMU_ADDR);
  mag.begin(MAG_ADDR);
  filter.begin(100);
  tsl.begin();
  tsl.setGain(TSL2591_GAIN_LOW);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  tsl.enableAutoGain(false);
}

void loop() {
  // Read raw sensor data
  imu.readAccelerometer();
  imu.readGyroscope();
  mag.readMagnetometer();
  uint16_t broadband, ir;
  tsl.getLuminosity(&broadband, &ir);

  // Convert sensor data to SI units
  float ax = imu.accelerometerData.x * 9.81;
  float ay = imu.accelerometerData.y * 9.81;
  float az = imu.accelerometerData.z * 9.81;
  float gx = imu.gyroscopeData.x * PI / 180;
  float gy = imu.gyroscopeData.y * PI / 180;
  float gz = imu.gyroscopeData.z * PI / 180;
  float mx = mag.magnetometerData.x;
  float my = mag.magnetometerData.y;
  float mz = mag.magnetometerData.z;
  float lux = tsl.calculateLux(broadband, ir);

  // Update orientation estimate with sensor data
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  // Get orientation as quaternion
  float q0 = filter.q0;
  float q1 = filter.q1;
  float q2 = filter.q2;
  float q3 = filter.q3;

  // Calculate position of sun relative to device
  float lat = 37.7749; // Replace with latitude of device
  float lon = -122.4194; // Replace with longitude of device
  float dec = asin(sin(-23.44 * PI / 180) * sin(2 * PI * (172 - 81) / 365)); // Approximate declination angle for date
  float hourAngle = acos(cos((90.833 * PI / 180) / (cos(lat) * cos(dec)) - tan(lat) * tan(dec))); // Calculate hour angle
  if (isnan(hourAngle)) {
    Serial.println("Sun is below the horizon");
  } else {
    float azim = acos((sin(lat) * cos(dec) - cos(lat) * sin(dec) * cos(hourAngle)) / cos(90 - lat)); // Calculate azimuth angle
    if (sin(hourAngle) > 0) {
      azim = 2 * PI - azim;
    }
    float sunPitch = asin(cos(dec) * sin(hourAngle)); // Calculate pitch angle
    float sunRoll = atan2(-sin(dec) * cos(hourAngle), cos(dec) * sin(lat) - sin(dec) * cos(lat) * cos(hourAngle)); // Calculate roll
