#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const char* ssid = "Plii";
const char* password = "bismillah2025sukses";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); 

float angleLeft = 0.0;
float forceLeft = 0.0;
float angleRight = 0.0;
float forceRight = 0.0;

bool leftActive = false;
bool rightActive = false;

struct Leg {
    int ch[3];
    bool state;
    float offset_x;
    float offset_y;
    float offset_z;
    float rot_z;
    float P_ori[4][3];
    float P[4][3];
    Adafruit_PWMServoDriver *pwm;
};

void handleWebSocketMessage(String msg) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, msg);
  
    if (error) {
      Serial.print("JSON Parsing error: ");
      Serial.println(error.c_str());
      return;
    }
  
    if (!doc["left"].isNull()) {
      angleLeft = doc["left"]["angle"].as<float>();
      forceLeft = doc["left"]["force"].as<float>();
      leftActive = true;
    } else {
      angleLeft = 0.0;
      forceLeft = 0.0;
      leftActive = false;
    }
  
    if (!doc["right"].isNull()) {
      angleRight = doc["right"]["angle"].as<float>();
      forceRight = doc["right"]["force"].as<float>();
      rightActive = true;
    } else {
      angleRight = 0.0;
      forceRight = 0.0;
      rightActive = false;
    }
  
    Serial.printf("LEFT: angle=%.1f, force=%.2f | RIGHT: angle=%.1f, force=%.2f\n",
                  angleLeft, forceLeft, angleRight, forceRight);
}
  

void onWebSocketEvent(AsyncWebSocket *server,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0; 
      String msg = (char*)data;
      handleWebSocketMessage(msg);
    }
  }
}

Adafruit_PWMServoDriver pwm_ll = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm_rl = Adafruit_PWMServoDriver(0x41);


Leg L1 = {{0, 1, 2}, true, -63.0, 83.5, -20.0, 45.0, {{-150, 110, -100}, {-150, 130, -80}, {-150, 190, -80}, {-150, 210, -100}}, {{-150, 110, -100}, {-150, 130, -80}, {-150, 190, -80}, {-150, 210, -100}}, &pwm_ll};
Leg L2 = {{4, 5, 6}, false, -81.5, 0.0, -20.0, 0.0, {{-180, -50, -100}, {-180, -30, -80}, {-180, 30, -80}, {-180, 50, -100}}, {{-180, -50, -100}, {-180, -30, -80}, {-180, 30, -80}, {-180, 50, -100}}, &pwm_ll};
Leg L3 = {{8, 9, 10}, true, -63.0, -83.5, -20.0, -45.0, {{-150, -210, -100}, {-150, -190, -80}, {-150, -130, -80}, {-150, -110, -100}}, {{-150, -210, -100}, {-150, -190, -80}, {-150, -130, -80}, {-150, -110, -100}},  &pwm_ll};

Leg R1 = {{0, 1, 2}, false, 63.0, 83.5, -20.0, -45.0, {{150, 110, -100}, {150, 130, -80}, {150, 190, -80}, {150, 210, -100}}, {{150, 110, -100}, {150, 130, -80}, {150, 190, -80}, {150, 210, -100}}, &pwm_rl};
Leg R2 = {{4, 5, 6}, true, 81.5, 0.0, -20.0, 0.0, {{180, -50, -100}, {180, -30, -80}, {180, 30, -80}, {180, 50, -100}}, {{180, -50, -100}, {180, -30, -80}, {180, 30, -80}, {180, 50, -100}}, &pwm_rl};
Leg R3 = {{8, 9, 10}, false, 63.0, -83.5, -20.0, 45.0, {{150, -210, -100}, {150, -190, -80}, {150, -130, -80}, {150, -110, -100}}, {{150, -210, -100}, {150, -190, -80}, {150, -130, -80}, {150, -110, -100}}, &pwm_rl};

const int SERVOMIN = 102; 
const int SERVOMAX = 512;

const float coxa_len  = 43.0;
const float femur_len = 60.0;
const float tibia_len = 104.0;

const double phi = 3.14159265358979323846;

float alpha = 0; 
float beta = 0;   
float gamma1 = 0; 

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

void bodyKinematic(Leg &leg, float &x, float &y, float &z) {
  float x_global = x + leg.offset_x;
  float y_global = y + leg.offset_y;
  float z_global = z + leg.offset_z;

  float xr = x_global;
  float yr = y_global;
  float zr = z_global;

  float gamma_rad = radians(gamma1);

  float tempX = xr * cos(gamma_rad) - yr * sin(gamma_rad);
  float tempY = xr * sin(gamma_rad) + yr * cos(gamma_rad);
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

void kakiKiri(Leg &leg, float x, float y, float z, float &servoCoxa, float &servoFemur, float &servoTibia) {
  float x_rot, y_rot;
  rotateZ(x, y, leg.rot_z, x_rot, y_rot);

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

void kakiKanan(Leg &leg, float x, float y, float z, float &servoCoxa, float &servoFemur, float &servoTibia) {
    float x_rot, y_rot;
    rotateZ(x, y, leg.rot_z, x_rot, y_rot);

    float c, f, t;
    inverseKinematic(leg, x_rot, y_rot, z, c, f, t);
    
    servoCoxa  = constrainAngle(90.0 - c);
    servoFemur = constrainAngle(90.0 + f);
    servoTibia = constrainAngle(90.0 - t);
}

int angleToPulse(float angle) {
  return SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN);
}

float y_L1, y_L2, y_L3, y_R1, y_R2, y_R3;

void doLeg(Leg &leg, float t, float &y) {
    float servoCoxa, servoFemur, servoTibia;
    
    if(leg.state) {
      float x_global = leg.P[0][0];
      float z_global = leg.P[0][2];
      float stepSize = (leg.P[0][1] - leg.P[3][1]) / steps;
      
      float x_local = x_global - leg.offset_x;
      float y_local = y - leg.offset_y;
      float z_local = z_global - leg.offset_z;


      if (leg.pwm == &pwm_ll) {
        kakiKiri(leg, x_local, y_local, z_local, servoCoxa, servoFemur, servoTibia);
      } else {
        kakiKanan(leg, x_local, y_local, z_local, servoCoxa, servoFemur, servoTibia);
      }
      y += stepSize; 
    } else { 
      float x_global, y_swing_global, z_global;
      bezier(leg, t, x_global, y_swing_global, z_global);

      float x_local = x_global - leg.offset_x;
      float y_local = y_swing_global - leg.offset_y;
      float z_local = z_global - leg.offset_z;
      
      if (leg.pwm == &pwm_ll) {
        kakiKiri(leg, x_local, y_local, z_local, servoCoxa, servoFemur, servoTibia);
      } else {
        kakiKanan(leg, x_local, y_local, z_local, servoCoxa, servoFemur, servoTibia);
      }
    }
    
    leg.pwm->setPWM(leg.ch[0], 0, angleToPulse(servoCoxa));
    leg.pwm->setPWM(leg.ch[1], 0, angleToPulse(servoFemur));
    leg.pwm->setPWM(leg.ch[2], 0, angleToPulse(servoTibia));
}

void rotateBezier(Leg &leg, float angle_deg) {
    float pivot_x = leg.P_ori[0][0];
    float pivot_y = (leg.P_ori[0][1] + leg.P_ori[3][1]) / 2.0;
    float rad = -angle_deg * phi / 180.0;
    float cos_t = cos(rad);
    float sin_t = sin(rad);

    for (int i = 0; i < 4; i++) {
        float x = leg.P_ori[i][0];
        float y = leg.P_ori[i][1];
        float z = leg.P_ori[i][2];
        
        float x_trans = x - pivot_x;
        float y_trans = y - pivot_y;

        float x_rot = x_trans * cos_t - y_trans * sin_t;
        float y_rot = x_trans * sin_t + y_trans * cos_t;

        leg.P[i][0] = x_rot + pivot_x;
        leg.P[i][1] = y_rot + pivot_y;
        leg.P[i][2] = z;
    }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan ke WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Terhubung. IP: " + WiFi.localIP().toString());

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.begin();

  Serial.println("Hexapod");
  pwm_ll.begin();
  pwm_ll.setPWMFreq(50);
  pwm_rl.begin();     
  pwm_rl.setPWMFreq(50);

  float servoCoxa, servoFemur, servoTibia;
  
  kakiKiri(L1, -150.0 - L1.offset_x, 130.0 - L1.offset_y, -100.0 - L1.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm_ll.setPWM(L1.ch[0], 0, angleToPulse(servoCoxa));

  kakiKiri(L2, -180.0 - L2.offset_x, 0.0 - L2.offset_y, -100.0 - L2.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm_ll.setPWM(L2.ch[0], 0, angleToPulse(servoCoxa));

  kakiKiri(L3, -150.0 - L3.offset_x, -130.0 - L3.offset_y, -100.0 - L3.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm_ll.setPWM(L3.ch[0], 0, angleToPulse(servoCoxa));

  kakiKanan(R1, 150.0 - R1.offset_x, 130.0 - R1.offset_y, -100.0 - R1.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm_rl.setPWM(R1.ch[0], 0, angleToPulse(servoCoxa));

  kakiKanan(R2, 180.0 - R2.offset_x, 0.0 - R2.offset_y, -100.0 - R2.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm_rl.setPWM(R2.ch[0], 0, angleToPulse(servoCoxa));

  kakiKanan(R3, 150.0 - R3.offset_x, -130.0 - R3.offset_y, -100.0 - R3.offset_z, servoCoxa, servoFemur, servoTibia);
  pwm_rl.setPWM(R3.ch[0], 0, angleToPulse(servoCoxa));

  delay(3000);
}

void loop() {

  if (forceLeft > 0.0) {
    float direction;
    if (angleLeft >= 315 || angleLeft < 45) {
      direction = 0.0; // Depan
  } else if (angleLeft >= 45 && angleLeft < 135) {
      direction = 90.0; // Kiri
  } else if (angleLeft >= 135 && angleLeft < 225) {
      direction = 180.0; // Belakang
  } else { 
      direction = 270.0; // Kanan
  }
      
    rotateBezier(L1, direction);
    rotateBezier(L2, direction);
    rotateBezier(L3, direction);
    rotateBezier(R1, direction);
    rotateBezier(R2, direction);
    rotateBezier(R3, direction);

    y_L1 = L1.state ? L1.P[3][1] : L1.P[0][1]; 
    y_L2 = L2.state ? L2.P[3][1] : L2.P[0][1];
    y_L3 = L3.state ? L3.P[3][1] : L3.P[0][1];

    y_R1 = R1.state ? R1.P[3][1] : R1.P[0][1]; 
    y_R2 = R2.state ? R2.P[3][1] : R2.P[0][1];
    y_R3 = R3.state ? R3.P[3][1] : R3.P[0][1];

    for(int i=0; i<=steps; i++) {
      float t = (float)i/steps;
      
      doLeg(L1, t, y_L1);
      doLeg(L2, t, y_L2);
      doLeg(L3, t, y_L3);

      doLeg(R1, t, y_R1);
      doLeg(R2, t, y_R2);
      doLeg(R3, t, y_R3);
      
      delay(10);
    }

    L1.state = !L1.state;
    L2.state = !L2.state;
    L3.state = !L3.state;

    R1.state = !R1.state;
    R2.state = !R2.state;
    R3.state = !R3.state;
  }   
}
