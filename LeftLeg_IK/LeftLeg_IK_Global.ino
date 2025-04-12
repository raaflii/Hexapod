#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct Leg {
    int ch[3];
    bool state;
    float offset_x;
    float offset_y;
    float offset_z;
};

Leg L1 = {{0, 1, 2}, true, -63.0, 83.5, -20.0};
Leg L2 = {{4, 5, 6}, false, -81.5, 0.0, -20.0};
Leg L3 = {{8, 9, 10}, true, -63.0, -83.5, -20.0};

const int SERVOMIN = 102; 
const int SERVOMAX = 512;

const float coxa_len  = 43.0;
const float femur_len = 60.0;
const float tibia_len = 104.0;

float constrainAngle(float val) {
  return constrain(val, 20, 180);
}

void inverseKinematic(Leg &leg, float x, float y, float z, float &sudutCoxa, float &sudutFemur, float &sudutTibia) {
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

void kakiKiri(Leg &leg, float x, float y, float z, float &servoCoxa, float &servoFemur, float &servoTibia) {
  float c, f, t;
  inverseKinematic(leg, x, y, z, c, f, t);

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
  Serial.println("Masukkan input global kaki, x, y, z (misal: L1 100 30 0):");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
}

void loop() {
  if (Serial.available() > 0) {
    String Strleg = Serial.readStringUntil(' ');
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();
    
    while (Serial.available() && Serial.read() != '\n') { }

    Serial.print("x_local=");
    Serial.print(x);
    Serial.print(", y_local=");
    Serial.print(y);
    Serial.print(", z_local=");
    Serial.println(z);

    
    float servoCoxa, servoFemur, servoTibia;

    Leg legs[3] = {L1, L2, L3};
    Leg leg;

    if (Strleg == "L1") {
        leg = legs[0];
    } else if (Strleg == "L2") {
        leg = legs[1];
    } else if (Strleg == "L3") {
        leg = legs[2];
    } else { 
        Serial.println("Ga valid bg");
        return;
    }

    x -= leg.offset_x;
    y -= leg.offset_y;
    z -= leg.offset_z;

    kakiKiri(leg, x, y, z, servoCoxa, servoFemur, servoTibia);
    
    Serial.print("Servo Coxa: ");
    Serial.println(servoCoxa);
    Serial.print("Servo Femur: ");
    Serial.println(servoFemur);
    Serial.print("Servo Tibia: ");
    Serial.println(servoTibia);
    
    int pulseCoxa  = angleToPulse(servoCoxa);
    int pulseFemur = angleToPulse(servoFemur);
    int pulseTibia = angleToPulse(servoTibia);
    
    pwm.setPWM(leg.ch[0], 0, pulseCoxa);
    pwm.setPWM(leg.ch[1], 0, pulseFemur);
    pwm.setPWM(leg.ch[2], 0, pulseTibia);
  }
  delay(10);
}
