#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ===== MOTORS =====
Servo mFL, mFR, mRL, mRR;

#define PIN_FL 12
#define PIN_RL 13
#define PIN_RR 14
#define PIN_FR 15

// ===== CONTROL DATA =====
typedef struct {
  int throttle;
  int yaw;
  bool armed;
} ControlPacket;

ControlPacket rx;

// ===== CALLBACK =====
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  memcpy(&rx, data, sizeof(rx));
}

void setup() {
  Serial.begin(115200);

  // Attach ESCs (50 Hz servo PWM)
  mFL.attach(PIN_FL, 1000, 2000);
  mFR.attach(PIN_FR, 1000, 2000);
  mRL.attach(PIN_RL, 1000, 2000);
  mRR.attach(PIN_RR, 1000, 2000);

  // Send minimum throttle for ESC arming
  mFL.writeMicroseconds(1000);
  mFR.writeMicroseconds(1000);
  mRL.writeMicroseconds(1000);
  mRR.writeMicroseconds(1000);

  delay(3000); // ⚠️ ESC ARM TIME

  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onReceive);

  Serial.println("Drone ready — ESCs armed");
}

void loop() {
  if (!rx.armed) {
    // DISARM
    mFL.writeMicroseconds(1000);
    mFR.writeMicroseconds(1000);
    mRL.writeMicroseconds(1000);
    mRR.writeMicroseconds(1000);
    return;
  }

  int base = rx.throttle;
  int yaw  = rx.yaw;

  mFL.writeMicroseconds(constrain(base - yaw, 1000, 2000));
  mFR.writeMicroseconds(constrain(base + yaw, 1000, 2000));
  mRL.writeMicroseconds(constrain(base - yaw, 1000, 2000));
  mRR.writeMicroseconds(constrain(base + yaw, 1000, 2000));

  delay(5);
}
