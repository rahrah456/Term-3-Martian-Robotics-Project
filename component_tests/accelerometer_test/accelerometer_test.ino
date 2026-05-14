#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(10000);
  delay(100);

  if (!imu.init()) {
    Serial.println("Failed to detect LSM6DS33!");
    while (1);
  }

  imu.enableDefault();

  Serial.println("AltIMU-10 v6 initialized");
}

void loop() {
  float ax, ay, az;

  // Read data using function
  readAcceleration(ax, ay, az);

  Serial.print("Ax: "); Serial.print(ax); Serial.print(" g\t");
  Serial.print("Ay: "); Serial.print(ay); Serial.print(" g\t");
  Serial.print("Az: "); Serial.print(az); Serial.println(" g");

  delay(100);
}

void readAcceleration(float &ax, float &ay, float &az) {
  imu.read();

  ax = imu.a.x / 32768.0 * 2.0;  // ±2g scale
  ay = imu.a.y / 32768.0 * 2.0;
  az = imu.a.z / 32768.0 * 2.0;
}