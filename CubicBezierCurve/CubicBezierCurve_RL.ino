#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int R1[3] = {0, 1, 2}; 
const int R2[3] = {4, 5, 6}; 
const int R3[3] = {8, 9, 10}; 

const int SERVOMIN = 102; 
const int SERVOMAX = 512;

const float coxa_len  = 43.0;
const float femur_len = 60.0;
const float tibia_len = 104.0;

float P0[3] = {80, -50, -50};  
float P1[3] = {80, -30, -30};  
float P2[3] = {80, 30, -30};  
float P3[3] = {80, 50, -50};

float constrainAngle(float val) {
  return constrain(val, 0, 180);
}

void bezier(float t, float P0[3], float P1[3], float P2[3], float P3[3], float &x, float &y, float &z) {
  float u = 1 - t;
  x = u*u*u * P0[0] + 3 * u*u * t * P1[0] + 3 * u * t*t * P2[0] + t*t*t * P3[0];
  y = u*u*u * P0[1] + 3 * u*u * t * P1[1] + 3 * u * t*t * P2[1] + t*t*t * P3[1];
  z = u*u*u * P0[2] + 3 * u*u * t * P1[2] + 3 * u * t*t * P2[2] + t*t*t * P3[2];
}

void inverseKinematic(float x, float y, float z, float &sudutCoxa, float &sudutFemur, float &sudutTibia) {
  sudutCoxa = degrees(atan2(y, x));
  float jarakHorizontal = sqrt(x * x + y * y) - coxa_len;
  float femur_1 = degrees(atan2(z, jarakHorizontal));
  float a = sqrt(z * z + jarakHorizontal * jarakHorizontal);
  a = min(a, femur_len + tibia_len);
  a = max(a, fabs(femur_len - tibia_len));
  
  float ratio = (femur_len * femur_len + a * a - tibia_len * tibia_len) / (2.0 * a * femur_len);
  ratio = constrain(ratio, -1, 1);
  float femur_2 = degrees(acos(ratio));
  
  sudutFemur = femur_1 + femur_2;
  
  float ratio2 = (femur_len * femur_len + tibia_len * tibia_len - a * a) / (2.0 * femur_len * tibia_len);
  ratio2 = constrain(ratio2, -1, 1);
  sudutTibia = degrees(acos(ratio2)) - 90.0;
}

void kakiKanan(float x, float y, float z, float &servoCoxa, float &servoFemur, float &servoTibia) {
  float c, f, t;
  inverseKinematic(x, y, z, c, f, t);

  servoCoxa  = constrainAngle(90.0 - c);
  servoFemur = constrainAngle(90.0 + f);
  servoTibia = constrainAngle(90.0 - t);
}

void kakiKiri(float x, float y, float z, float &servoCoxa, float &servoFemur, float &servoTibia) {
  float c, f, t;
  inverseKinematic(x, y, z, c, f, t);

  servoCoxa  = constrainAngle(270.0 - c);
  servoFemur = constrainAngle(90.0 - f);
  servoTibia = constrainAngle(90.0 + t);
}

int angleToPulse(float angle) {
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Cubic Bezier Curve");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
}

void loop() {
  int steps = 20;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    float x, y, z;
    bezier(t, P0, P1, P2, P3, x, y, z);

    float servoCoxa, servoFemur, servoTibia;
    kakiKanan(x, y, z, servoCoxa, servoFemur, servoTibia);

    int pulseCoxa  = angleToPulse(servoCoxa);
    int pulseFemur = angleToPulse(servoFemur);
    int pulseTibia = angleToPulse(servoTibia);

    pwm.setPWM(R1[0], 0, pulseCoxa);
    pwm.setPWM(R1[1], 0, pulseFemur);
    pwm.setPWM(R1[2], 0, pulseTibia);

    pwm.setPWM(R2[0], 0, pulseCoxa);
    pwm.setPWM(R2[1], 0, pulseFemur);
    pwm.setPWM(R2[2], 0, pulseTibia);

    pwm.setPWM(R3[0], 0, pulseCoxa);
    pwm.setPWM(R3[1], 0, pulseFemur);
    pwm.setPWM(R3[2], 0, pulseTibia);

    Serial.print("Bezier t=");
    Serial.print(t);
    Serial.print(" -> x=");
    Serial.print(x);
    Serial.print(", y=");
    Serial.print(y);
    Serial.print(", z=");
    Serial.println(z);

    delay(50);
  }
}