#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SimpleKalmanFilter.h>

Adafruit_MPU6050 mpu;

SimpleKalmanFilter kalmanRoll(2, 2, 0.01);  
SimpleKalmanFilter kalmanPitch(2, 2, 0.01);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Inisialisasi MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 tidak terdeteksi. Periksa sambungan!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 siap digunakan");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float roll  = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  float rollFiltered  = kalmanRoll.updateEstimate(roll);
  float pitchFiltered = kalmanPitch.updateEstimate(pitch);

  Serial.print("Roll: ");
  Serial.print(rollFiltered);
  Serial.print(" °\tPitch: ");
  Serial.print(pitchFiltered);
  Serial.println(" °");

  delay(5);  // 50 Hz refresh rate
}
