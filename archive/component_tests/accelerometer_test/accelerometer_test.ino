#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

void setup() {
  Serial.begin(115200);

  // CRITICAL FOR ARDUINO GIGA: 
  // Wait for the Serial Monitor to open before running the rest of the code
  while (!Serial) {
    delay(10);
  }

  Serial.println("Starting I2C and looking for IMU...");
  
  // The Arduino GIGA uses pins 20 (SDA) and 21 (SCL) for default Wire
  Wire.begin();

  // Initialize the LSM6DS33 accelerometer
  if (!imu.init()) {
    Serial.println("Failed to detect IMU! Double-check wiring.");
    while (1); // Freeze here if the sensor isn't found
  }

  // enableDefault() sets the accelerometer to +/- 2g full scale
  imu.enableDefault();
  
  Serial.println("IMU initialized successfully. Reading Accelerometer...");
}

void loop() {
  // readAcc() ONLY queries the accelerometer registers, skipping the gyro
  imu.readAcc();

  // Convert raw 16-bit integer data to G-forces
  // The default scale is +/- 2g, so we divide by 32768.0 and multiply by 2.0
  float ax = (imu.a.x / 32768.0) * 2.0;
  float ay = (imu.a.y / 32768.0) * 2.0;
  float az = (imu.a.z / 32768.0) * 2.0;

  // Print the values with 3 decimal places of precision
  Serial.print("Ax: "); Serial.print(ax, 3); Serial.print(" g\t");
  Serial.print("Ay: "); Serial.print(ay, 3); Serial.print(" g\t");
  Serial.print("Az: "); Serial.print(az, 3); Serial.println(" g");

  // Read 10 times a second
  delay(100);
}