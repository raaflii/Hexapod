#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

struct Leg {
    int ch[3];
    bool state;
    float offset_x;
    float offset_y;
    float offset_z;
    float rot_z;
    float P[4][3];   
};

Leg L1 = {{0, 1, 2}, true, -63.0, 83.5, -20.0, 45.0, {{-160, 110, -70}, {-160, 130, -55}, {-160, 190, -55}, {-160, 210, -70}}};
Leg L2 = {{4, 5, 6}, false, -81.5, 0.0, -20.0, 0.0, {{-190, -50, -70}, {-190, -30, -55}, {-190, 30, -55}, {-190, 50, -70}}};
Leg L3 = {{8, 9, 10}, true, -63.0, -83.5, -20.0, -45.0, {{-160, -210, -70}, {-160, -190, -55}, {-160, -130, -55}, {-160, -110, -70}}};

const int SERVOMIN = 102; 
const int SERVOMAX = 512;

const float coxa_len  = 43.0;
const float femur_len = 60.0;
const float tibia_len = 104.0;

int steps = 30;

float constrainAngle(float val) {
  return constrain(val, 20, 180);
}

void bezier(Leg &leg, float t, float &x, float &y, float &z) {
    float u = 1 - t;
    x = u*u*u * leg.P[0][0] + 3 * u*u * t * leg.P[1][0] + 3 * u * t*t * leg.P[2][0] + t*t*t * leg.P[3][0];
    y = u*u*u * leg.P[0][1] + 3 * u*u * t * leg.P[1][1] + 3 * u * t*t * leg.P[2][1] + t*t*t * leg.P[3][1];
    z = u*u*u * leg.P[0][2] + 3 * u*u * t * leg.P[1][2] + 3 * u * t*t * leg.P[2][2] + t*t*t * leg.P[3][2];
}

void rotateZ(float x_in, float y_in, float theta_deg, float &x_out, float &y_out) {
  float theta_rad = radians(theta_deg);
  x_out = x_in * cos(theta_rad) - y_in * sin(theta_rad);
  y_out = x_in * sin(theta_rad) + y_in * cos(theta_rad);
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
  float x_rot, y_rot;
  rotateZ(x, y, leg.rot_z, x_rot, y_rot);

  Serial.print("x_rot: "); Serial.println(x_rot);
  Serial.print("y_rot: "); Serial.println(y_rot);

  float c, f, t;
  inverseKinematic(leg, x_rot, y_rot, z, c, f, t);

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

float y_L1, y_L2, y_L3;

void doLeg(Leg &leg, float t, float &y) {
    float servoCoxa, servoFemur, servoTibia;
    
    if(leg.state) {
      float x_global = leg.P[0][0];
      float z_global = leg.P[0][2];
      float stepSize = (leg.P[0][1] - leg.P[3][1]) / steps;
      
      float x_local = x_global - leg.offset_x;
      float y_local = y - leg.offset_y;
      float z_local = z_global - leg.offset_z;
      
      kakiKiri(leg, x_local, y_local, z_local, servoCoxa, servoFemur, servoTibia);
      y += stepSize; 
    } else { 
      float x_global, y_swing_global, z_global;
      bezier(leg, t, x_global, y_swing_global, z_global);

      float x_local = x_global - leg.offset_x;
      float y_local = y_swing_global - leg.offset_y;
      float z_local = z_global - leg.offset_z;
      
      kakiKiri(leg, x_local, y_local, z_local, servoCoxa, servoFemur, servoTibia);
    }
    
    pwm.setPWM(leg.ch[0], 0, angleToPulse(servoCoxa));
    pwm.setPWM(leg.ch[1], 0, angleToPulse(servoFemur));
    pwm.setPWM(leg.ch[2], 0, angleToPulse(servoTibia));
  }

void setup() {
  Serial.begin(115200);
  Serial.println("CBC Global LL");
  pwm.begin();
  pwm.setPWMFreq(50);

  float servoCoxa, servoFemur, servoTibia;
  
  kakiKiri(L1, -130.0 - L1.offset_x, 130.0 - L1.offset_y, -70.0 - L1.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm.setPWM(L1.ch[0], 0, angleToPulse(servoCoxa));
  delay(500);

  kakiKiri(L2, -160.0 - L2.offset_x, 0.0 - L2.offset_y, -70.0 - L2.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm.setPWM(L2.ch[0], 0, angleToPulse(servoCoxa));
  delay(500);

  kakiKiri(L3, -130.0 - L3.offset_x, -130.0 - L3.offset_y, -70.0 - L3.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm.setPWM(L3.ch[0], 0, angleToPulse(servoCoxa));

  delay(3000);
}

void loop() {
    while(true) {
        y_L1 = L1.state ? L1.P[3][1] : L1.P[0][1]; 
        y_L2 = L2.state ? L2.P[3][1] : L2.P[0][1];
        y_L3 = L3.state ? L3.P[3][1] : L3.P[0][1];
    
        for(int i=0; i<=steps; i++) {
          float t = (float)i/steps;
          
          doLeg(L1, t, y_L1);
          doLeg(L2, t, y_L2);
          doLeg(L3, t, y_L3);
          
          delay(10);
        }
    
        L1.state = !L1.state;
        L2.state = !L2.state;
        L3.state = !L3.state;
      }
}
