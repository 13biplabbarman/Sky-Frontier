#include <WiFi.h>
#include <esp_now.h>

// ===== JOYSTICK PINS =====
#define JOY_Y 35   // Left joystick vertical (Throttle)
#define JOY_X 34   // Left joystick horizontal (Yaw)

// ===== BUTTON =====
#define ARM_BUTTON 18   // S6 (Emergency Stop)

// ===== JOYSTICK CALIB =====
#define JOY_CENTER 2048
#define DEADZONE   80

// ===== THROTTLE LIMITS =====
int throttle = 1100;        // Initial idle
#define THROTTLE_MIN 1000
#define THROTTLE_MAX 1700

bool armed = false;

// ===== DATA PACKET =====
typedef struct {
  int throttle;
  int yaw;
  bool armed;
} ControlPacket;

ControlPacket packet;

// ===== DRONE MAC (REPLACE WITH RX MAC) =====
uint8_t droneMAC[] = {0x24,0x6F,0x28,0xAA,0xBB,0xCC};

void setup() {
  Serial.begin(115200);

  pinMode(ARM_BUTTON, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, droneMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  Serial.println("Controller ready");
}

void loop() {
  // ----- ARM / DISARM -----
  static bool lastBtn = HIGH;
  bool btn = digitalRead(ARM_BUTTON);

  if (lastBtn == HIGH && btn == LOW) {
    armed = !armed;
    Serial.println(armed ? "ARMED" : "DISARMED");
    delay(300);
  }
  lastBtn = btn;

  // ----- THROTTLE CONTROL -----
  int joyY = analogRead(JOY_Y) - JOY_CENTER;

  if (abs(joyY) > DEADZONE) {
    throttle += joyY / 300;   // Incremental change
    throttle = constrain(throttle, THROTTLE_MIN, THROTTLE_MAX);
  }

  // ----- YAW CONTROL -----
  int joyX = analogRead(JOY_X) - JOY_CENTER;
  int yaw = (abs(joyX) > DEADZONE) ? joyX / 10 : 0;

  // ----- PACKET -----
  packet.throttle = throttle;
  packet.yaw = yaw;
  packet.armed = armed;

  esp_now_send(droneMAC, (uint8_t *)&packet, sizeof(packet));

  delay(20); // 50 Hz
}

Serial.println(armed ? "ARMED SENT" : "DISARMED SENT");
