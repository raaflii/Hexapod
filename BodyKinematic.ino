#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// true = do stance phase, false = do swing phase
struct Leg {
  int ch[3];
  bool state;
  float offset_x;
  float offset_y;
  float offset_z;
};

Leg R1 = {{0, 1, 2}, true, 63.0, 83.5, -20.0};
Leg R2 = {{4, 5, 6}, false, 81.5, 0.0, -20.0};
Leg R3 = {{8, 9, 10}, true, 63.0, -83.5, -20.0};

float alpha = 0; 
float beta = 0;   
float gamma = 0; 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVOMIN = 102; 
const int SERVOMAX = 512;

const float coxa_len  = 43.0;
const float femur_len = 60.0;
const float tibia_len = 104.0;

float P0[3] = {80, -50, -50};  
float P1[3] = {80, -30, -30};  
float P2[3] = {80, 30, -30};  
float P3[3] = {80, 50, -50};

int steps = 30;

void bodyKinematic(Leg &leg, float &x, float &y, float &z) {
  float x_global = x + leg.offset_x;
  float y_global = y + leg.offset_y;
  float z_global = z + leg.offset_z;

  float xr = x_global;
  float yr = y_global;
  float zr = z_global;

  float tempX = xr * cos(gamma) - yr * sin(gamma);
  float tempY = xr * sin(gamma) + yr * cos(gamma);
  xr = tempX;
  yr = tempY;

  tempX = xr * cos(beta) + zr * sin(beta);
  float tempZ = -xr * sin(beta) + zr * cos(beta);
  xr = tempX;
  zr = tempZ;

  tempY = yr * cos(alpha) - zr * sin(alpha);
  tempZ = yr * sin(alpha) + zr * cos(alpha);
  yr = tempY;
  zr = tempZ;

  x = xr - leg.offset_x;
  y = yr - leg.offset_y;
  z = zr - leg.offset_z;
}


float constrainAngle(float val) {
  return constrain(val, 0, 180);
}

void bezier(float t, float P0[3], float P1[3], float P2[3], float P3[3], float &x, float &y, float &z) {
  float u = 1 - t;
  x = u*u*u * P0[0] + 3 * u*u * t * P1[0] + 3 * u * t*t * P2[0] + t*t*t * P3[0];
  y = u*u*u * P0[1] + 3 * u*u * t * P1[1] + 3 * u * t*t * P2[1] + t*t*t * P3[1];
  z = u*u*u * P0[2] + 3 * u*u * t * P1[2] + 3 * u * t*t * P2[2] + t*t*t * P3[2];
}

void inverseKinematic(Leg &leg, float x, float y, float z, float &sudutCoxa, float &sudutFemur, float &sudutTibia) {

  bodyKinematic(leg ,x, y, z);

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

void kakiKanan(Leg &leg, float x, float y, float z, float &servoCoxa, float &servoFemur, float &servoTibia) {
  float c, f, t;
  inverseKinematic(leg, x, y, z, c, f, t);
  
  servoCoxa  = constrainAngle(90.0 - c);
  servoFemur = constrainAngle(90.0 + f);
  servoTibia = constrainAngle(90.0 - t);
}

void kakiKiri(Leg &leg, float x, float y, float z, float &servoCoxa, float &servoFemur, float &servoTibia) {
  float c, f, t;
  inverseKinematic(leg, x, y, z, c, f, t);
  
  servoCoxa  = constrainAngle(270.0 - c);
  servoFemur = constrainAngle(90.0 - f);
  servoTibia = constrainAngle(90.0 + t);
}

int angleToPulse(float angle) {
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Testing Body Kinematic");
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  float servoCoxa, servoFemur, servoTibia;

  kakiKanan(R1, 80, 0, -50, servoCoxa, servoFemur, servoTibia);

  int pulseCoxa  = angleToPulse(servoCoxa);
  int pulseFemur = angleToPulse(servoFemur);
  int pulseTibia = angleToPulse(servoTibia);

  pwm.setPWM(R1.ch[0], 0, pulseCoxa);
  pwm.setPWM(R1.ch[1], 0, pulseFemur);
  pwm.setPWM(R1.ch[2], 0, pulseTibia);

  kakiKanan(R2, 80, 0, -50, servoCoxa, servoFemur, servoTibia);

  int pulseCoxa  = angleToPulse(servoCoxa);
  int pulseFemur = angleToPulse(servoFemur);
  int pulseTibia = angleToPulse(servoTibia);

  pwm.setPWM(R2.ch[0], 0, pulseCoxa);
  pwm.setPWM(R2.ch[1], 0, pulseFemur);
  pwm.setPWM(R2.ch[2], 0, pulseTibia);

  kakiKanan(R3, 80, 0, -50, servoCoxa, servoFemur, servoTibia);

  int pulseCoxa  = angleToPulse(servoCoxa);
  int pulseFemur = angleToPulse(servoFemur);
  int pulseTibia = angleToPulse(servoTibia);
  
  pwm.setPWM(R3.ch[0], 0, pulseCoxa);
  pwm.setPWM(R3.ch[1], 0, pulseFemur);
  pwm.setPWM(R3.ch[2], 0, pulseTibia);

  delay(3000);
}

float y_R1, y_R2, y_R3;

void doLeg(Leg &leg, float t, float &y) {
  float servoCoxa, servoFemur, servoTibia;
  
  if(leg.state) {
    float x = P0[0];
    float z = P0[2];
    float stepSize = (P0[1] - P3[1])/steps;
    
    kakiKanan(leg, x, y, z, servoCoxa, servoFemur, servoTibia);
    y += stepSize;
  } else { 
    float x, y_swing, z;
    bezier(t, P0, P1, P2, P3, x, y_swing, z);
    kakiKanan(leg, x, y_swing, z, servoCoxa, servoFemur, servoTibia);
  }
  
  pwm.setPWM(leg.ch[0], 0, angleToPulse(servoCoxa));
  pwm.setPWM(leg.ch[1], 0, angleToPulse(servoFemur));
  pwm.setPWM(leg.ch[2], 0, angleToPulse(servoTibia));
}

void loop() {
  while(true) {
    y_R1 = R1.state ? P3[1] : P0[1];
    y_R2 = R2.state ? P3[1] : P0[1];
    y_R3 = R3.state ? P3[1] : P0[1];

    for(int i=0; i<=steps; i++) {
      float t = (float)i/steps;
      
      doLeg(R1, t, y_R1);
      doLeg(R2, t, y_R2);
      doLeg(R3, t, y_R3);
      
      delay(10);
    }

    R1.state = !R1.state;
    R2.state = !R2.state;
    R3.state = !R3.state;
  }
}
