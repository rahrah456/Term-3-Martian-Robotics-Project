#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>

LIS3MDL mag;
LSM6 imu;

// ========================================================
// RE-CALIBRATED MICROTESLA PARAMETERS (49.1579 uT FIELD)
// ========================================================
const float hard_iron_bias_x = -92.425254;
const float hard_iron_bias_y = 74.120214;
const float hard_iron_bias_z = -1.234978;

const double soft_iron_bias_xx = 0.928803;
const double soft_iron_bias_xy = 0.023684;
const double soft_iron_bias_xz = 0.070527;

const double soft_iron_bias_yx = 0.023684;
const double soft_iron_bias_yy = 0.925277;
const double soft_iron_bias_yz = -0.020476;

const double soft_iron_bias_zx = 0.070527;
const double soft_iron_bias_zy = -0.020476;
const double soft_iron_bias_zz = 0.797892;

void setup()
{
  Serial.begin(115200); 
  Wire.begin();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
    while (1);
  }
  mag.enableDefault(); // Configures default ±4 Gauss scale range

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize LSM6 IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop()
{
  mag.read();
  imu.read();

  // Computes tilt-compensated heading assuming +X is the forward direction of your car
  float heading = computeEllipsoidHeading((LIS3MDL::vector<int>){1, 0, 0});

  Serial.print("Heading: ");
  Serial.print(heading, 2);
  Serial.println("°");
  
  delay(100);
}

/*
Overhauled Heading Algorithm:
1. Converts raw LSB inputs to microTeslas by dividing by 68.42.
2. Subtracts Hard-Iron bias from the microTesla data.
3. Multiplies by the Soft-Iron inverse scaling matrix.
4. Leverages IMU vector tracking to ensure pitch/roll shifts do not distort the heading.
*/
template <typename T> float computeEllipsoidHeading(LIS3MDL::vector<T> from)
{
  // 1. Convert raw LSB to microTesla (uT) units first
  float ut_x = (float)mag.m.x / 68.42;
  float ut_y = (float)mag.m.y / 68.42;
  float ut_z = (float)mag.m.z / 68.42;

  // 2. Shift data to remove constant magnetic offsets (Hard-Iron)
  float xm_off = ut_x - hard_iron_bias_x;
  float ym_off = ut_y - hard_iron_bias_y;
  float zm_off = ut_z - hard_iron_bias_z;

  // 3. Transform data with the 3x3 Inverse Matrix (Soft-Iron Correction)
  LIS3MDL::vector<float> cal_m;
  cal_m.x = (xm_off * soft_iron_bias_xx) + (ym_off * soft_iron_bias_yx) + (zm_off * soft_iron_bias_zx);
  cal_m.y = (xm_off * soft_iron_bias_xy) + (ym_off * soft_iron_bias_yy) + (zm_off * soft_iron_bias_zy);
  cal_m.z = (xm_off * soft_iron_bias_xz) + (ym_off * soft_iron_bias_yz) + (zm_off * soft_iron_bias_zz);

  // 4. Extract standard accelerometer gravity references from the LSM6 
  LIS3MDL::vector<float> a = {(float)imu.a.x, (float)imu.a.y, (float)imu.a.z};

  // 5. Mathematical 3D Space Projection Matrix 
  LIS3MDL::vector<float> E;
  LIS3MDL::vector<float> N;
  
  // Cross product of calibrated Magnetometer and Accelerometer gives East vector
  LIS3MDL::vector_cross(&cal_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  
  // Cross product of Accelerometer and East vector gives North vector
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  // 6. Project vehicle reference vector into the calculated horizontal ground plane
  float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  
  if (heading < 0) heading += 360;
  return heading;
}
