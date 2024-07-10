#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

double ax, ay, az; // Acceleration values
double gx, gy, gz; // Gyroscope values
double vx = 0, vy = 0; // Velocity values
double sx = 0, sy = 0; // Displacement values
double theta = 0; // Orientation (yaw) in radians
unsigned long prevTime = 0;

void setup() {
  Wire.begin();
  //mpu.initialize();
  Serial.println("Initializing MPU6050...");
  mpu.begin();
  mpu.calcGyroOffsets(true);

  Serial.println("MPU6050 initialized successfully!");
  Serial.begin(9600);
  prevTime = millis();
}

void loop() {
  // Read accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu.update();
  // Convert raw accelerometer data to m/s^2 (assuming the MPU6050 outputs in g's)
  ax = (ax / 16384.0) * 9.81;
  ay = (ay / 16384.0) * 9.81;
  az = (az / 16384.0) * 9.81;

  // Convert raw gyroscope data to rad/s (assuming the MPU6050 outputs in °/s)
  gx = (gx / 131.0) * (PI / 180.0);
  gy = (gy / 131.0) * (PI / 180.0);
  gz = (gz / 131.0) * (PI / 180.0);

  // Calculate time interval
  unsigned long currentTime = millis();
  double dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Integrate acceleration to get velocity
  vy += ay * dt;

  // Integrate velocity to get displacement
  sy += vy * dt;

  // Integrate gyroscope to get orientation (yaw)
  theta += gz * dt;

  // Compute new coordinates
  sx += sy * sin(theta);
  sy += sy * cos(theta);

  // Print coordinates
  Serial.print("X: "); Serial.print(sx);
  Serial.print(" | Y: "); Serial.println(sy);

  delay(100);
}
