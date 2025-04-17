#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

const int SERVOMIN = 102; 
const int SERVOMAX = 512;

int angleToPulse(float angle) {
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

void setup() {
  Serial.begin(115200);
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);

  float servoAngle = 90.0;
  int pulseAngle  = angleToPulse(servoAngle);

  
  pwm1.setPWM(0, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(1, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(2, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(3, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(4, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(5, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(6, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(7, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(8, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(9, 0, pulseAngle);
  delay(500);
  pwm1.setPWM(10, 0, pulseAngle);
  delay(500);

  pwm2.setPWM(0, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(1, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(2, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(3, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(4, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(5, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(6, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(7, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(8, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(9, 0, pulseAngle);
  delay(500);
  pwm2.setPWM(10, 0, pulseAngle);

  delay(10);
}

void loop() {

}
