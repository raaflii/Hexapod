#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "SSID";
const char* password = "######";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); 


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
      Serial.print("Data dari WebSocket: ");
      Serial.println(msg);

      if (msg.indexOf("left") != -1) {
        Serial.println("Gerakan joystick kiri");
      } else if (msg.indexOf("right") != -1) {
        Serial.println("Gerakan joystick kanan");
      }
    }
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
}

void loop() {
  
}
