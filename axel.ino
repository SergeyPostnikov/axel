#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

MPU6050 mpu;
KalmanFilter kalmanX;
KalmanFilter kalmanY;

void setup() {
  Wire.begin();
  mpu.initialize();
  kalmanX.init(0.01, 0.1);
  kalmanY.init(0.01, 0.1);
  Serial.begin(9600); // Настройка последовательного порта
  // Настройте параметры MPU6050, если это необходимо
}

void loop() {
  mpu.getMotion6();
  int16_t ax = mpu.getAccelerationX();
  int16_t ay = mpu.getAccelerationY();
  int16_t az = mpu.getAccelerationZ();
  int16_t gx = mpu.getRotationX();
  int16_t gy = mpu.getRotationY();
  int16_t gz = mpu.getRotationZ();

  // Рассчитайте углы pitch и roll из акселерометра
  double pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  double roll = atan2(ay, az) * RAD_TO_DEG;

  // Примените фильтр Калмана к углам
  double filteredPitch = kalmanX.update(pitch, gx);
  double filteredRoll = kalmanY.update(roll, gy);

  // Отправьте данные в последовательный порт (Serial Plotter)
  Serial.print(filteredPitch);
  Serial.print("\t");
  Serial.println(filteredRoll);

  delay(100); // Задержка для стабильного чтения данных
}
