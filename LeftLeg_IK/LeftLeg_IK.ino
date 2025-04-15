#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVOMIN = 102; 
const int SERVOMAX = 512;

const float coxa_len  = 43.0;
const float femur_len = 60.0;
const float tibia_len = 104.0;

float constrainAngle(float val) {
  return constrain(val, 20, 180);
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

void kakiKiri(float x, float y, float z, float &servoCoxa, float &servoFemur, float &servoTibia) {
  float c, f, t;
  inverseKinematic(x, y, z, c, f, t);

  c = fmod(c, 360);  
  if (c < 0) {
    c += 360;        
  }

  servoCoxa = constrainAngle(270.0 - c); 
  servoFemur = constrainAngle(90.0 - f);
  servoTibia = constrainAngle(90.0 + t);
}

int angleToPulse(float angle) {
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Masukkan input x, y, z (misal: 100 30 0):");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
}

void loop() {
  if (Serial.available() > 0) {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();
    
    while (Serial.available() && Serial.read() != '\n') { }
    
    Serial.print("Input: x=");
    Serial.print(x);
    Serial.print(", y=");
    Serial.print(y);
    Serial.print(", z=");
    Serial.println(z);
    
    float servoCoxa, servoFemur, servoTibia;
    kakiKiri(x, y, z, servoCoxa, servoFemur, servoTibia);
    
    Serial.print("Servo Coxa: ");
    Serial.println(servoCoxa);
    Serial.print("Servo Femur: ");
    Serial.println(servoFemur);
    Serial.print("Servo Tibia: ");
    Serial.println(servoTibia);
    
    int pulseCoxa  = angleToPulse(servoCoxa);
    int pulseFemur = angleToPulse(servoFemur);
    int pulseTibia = angleToPulse(servoTibia);
    
    pwm.setPWM(0, 0, pulseCoxa);
    pwm.setPWM(1, 0, pulseFemur);
    pwm.setPWM(2, 0, pulseTibia);
    delay(500);

    pwm.setPWM(4, 0, pulseCoxa);
    pwm.setPWM(5, 0, pulseFemur);
    pwm.setPWM(6, 0, pulseTibia);
    delay(500);

    pwm.setPWM(8, 0, pulseCoxa);
    pwm.setPWM(9, 0, pulseFemur);
    pwm.setPWM(10, 0, pulseTibia);
  }
  delay(10);
}
