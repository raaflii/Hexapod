#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVOMIN = 102; 
const int SERVOMAX = 512;

int angleToPulse(float angle) {
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);

  float servoAngle = 90.0;
  int pulseAngle  = angleToPulse(servoAngle);

  
  pwm.setPWM(0, 0, pulseAngle);
  delay(3000);
  pwm.setPWM(1, 0, pulseAngle);
  delay(3000);
  pwm.setPWM(2, 0, pulseAngle);
  delay(3000);
  pwm.setPWM(3, 0, pulseAngle);
  delay(3000);
  pwm.setPWM(4, 0, pulseAngle);
  delay(3000);
  pwm.setPWM(5, 0, pulseAngle);
  delay(3000);
  pwm.setPWM(6, 0, pulseAngle);
  delay(3000);
  pwm.setPWM(7, 0, pulseAngle);
  delay(3000);
  pwm.setPWM(8, 0, pulseAngle);

  delay(10);
}

void loop() {

}
