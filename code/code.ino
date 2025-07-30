#include "pinout.h"
#include "config.h"

#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <ArduinoEigenDense.h>
#include <BleMouse.h>
#include <Filters.h>

using namespace Eigen;

BMI270 imu;
BleMouse bleMouse(DEVICE_NAME, DEVICE_MANU, 100);
Matrix3f R_ag;
FilterOnePole lowpassX(LOWPASS, 1);
FilterOnePole lowpassY(LOWPASS, 1);

Matrix3f get_cross_matrix(Vector3f u) {
  Matrix3f m = Matrix3f::Zero(3, 3);
  m(0, 1) = -u(2); m(0, 2) = u(1); m(1, 2) = -u(0);
  m(1, 0) = u(2); m(2, 0) = -u(1); m(2, 1) = u(0);
  return m;
}

Matrix3f calibrate_with_g(Vector3f a_bar) {
  Vector3f g(1, 0, 0);
  Vector3f a_hat = a_bar.normalized();
  float theta = acos(a_hat.dot(g));
  Serial.printf("a_bar: [%f, %f, %f]\n", a_bar(0), a_bar(1), a_bar(2));
  Serial.printf("theta: %f\n", theta);

  Matrix3f k_x = get_cross_matrix(a_hat.cross(g));

  return Matrix3f::Identity(3, 3) +  sin(theta)*k_x + (1-cos(theta))*(k_x*k_x);
}

void setup() {
  Serial.begin(115200);
  Serial.println("started");
  pinMode(STATUS_LED, OUTPUT);
  pinMode(MS1, INPUT);
  pinMode(MS2, INPUT);
  pinMode(MS3, INPUT);

  Wire.begin(SDA, SCL);
  imu.beginI2C(IMU_ADDR);

  Vector3f a_bar(0, 0, 0);

  delay(250);
  Serial.println("calibrating");

  digitalWrite(STATUS_LED, HIGH);

  for (int i=0; i<CALIBRATION_CYCLES; i++) {
    imu.getSensorData();
    a_bar(0) += imu.data.accelX;
    a_bar(1) += imu.data.accelY;
    a_bar(2) += imu.data.accelZ;
    //Serial.printf("a_bar %d: [%f, %f, %f]\n", i, a_bar(0), a_bar(1), a_bar(2));
    delay(DELTA_C);
  }

  digitalWrite(STATUS_LED, LOW);

  a_bar /= (float)(CALIBRATION_CYCLES);
  R_ag = calibrate_with_g(a_bar);
  /*
  Serial.printf(
    "%f %f %f\n%f %f %f\n %f %f %f\n", 
    R_ag(0,0), R_ag(0, 1), R_ag(0, 2),
    R_ag(1,0), R_ag(1, 1), R_ag(1, 2),
    R_ag(2,0), R_ag(2, 1), R_ag(2, 2)
  );
  */
  bleMouse.begin();
}

float t_0 = millis();

void loop() {
  if (millis() - t_0 > DELTA_T) {
    imu.getSensorData();
    Vector3f omega(imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ);
    omega = R_ag*omega;
    lowpassX.input(omega(0));
    lowpassY.input(omega(2));
    /*
    Serial.print("x:");
    Serial.print(lowpassX.output());
    Serial.print(",");
    Serial.print("y:");
    Serial.println(lowpassY.output());
    */
    //printf("x: %f, y: %f\n", -omega(0), -omega(2));
    signed char dx = 0.25*lowpassX.output();
    signed char dy = 0.25*lowpassY.output();
    if (abs(dx) > 64) { dx = (dx/abs(dx))*64; }
    if (abs(dy) > 64) { dy = (dy/abs(dy))*64; }
    //Serial.printf("%d, %d\n", dx, dy);
    bleMouse.move(-dx, -dy, 0);
    t_0 = millis();
  }
}
