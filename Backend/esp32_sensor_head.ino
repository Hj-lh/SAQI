#include <ESP32Servo.h>

#define TRIG_PIN 5
#define ECHO_PIN 18
#define SERVO_PIN 19

const int SERVO_LEFT = 150;
const int SERVO_CENTER = 90;
const int SERVO_RIGHT = 30;
const int SERVO_SETTLE_MS = 350;

Servo headServo;

float readDistanceOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2.0;
}

float readDistance() {
  float total = 0;
  int count = 0;

  for (int i = 0; i < 3; i++) {
    float cm = readDistanceOnce();
    if (cm > 0) {
      total += cm;
      count++;
    }
    delay(30);
  }

  if (count == 0) return -1;
  return total / count;
}

String cmJson(float cm) {
  if (cm < 0) return "null";
  return String(cm, 1);
}

void centerServo() {
  headServo.write(SERVO_CENTER);
}

float lookAndRead(int angle) {
  headServo.write(angle);
  delay(SERVO_SETTLE_MS);
  return readDistance();
}

void sendDistance() {
  float cm = readDistance();
  Serial.println("{\"ok\":true,\"cm\":" + cmJson(cm) + "}");
}

void sendScan() {
  float front = lookAndRead(SERVO_CENTER);
  float right = lookAndRead(SERVO_RIGHT);
  float left = lookAndRead(SERVO_LEFT);
  centerServo();
  delay(SERVO_SETTLE_MS);

  String best = "right";
  if (left > right) best = "left";
  if (left < 0 && right < 0) best = "unknown";
  else if (left >= 0 && right < 0) best = "left";

  String json = "{";
  json += "\"ok\":true,";
  json += "\"front_cm\":" + cmJson(front) + ",";
  json += "\"right_cm\":" + cmJson(right) + ",";
  json += "\"left_cm\":" + cmJson(left) + ",";
  json += "\"best\":\"" + best + "\"";
  json += "}";
  Serial.println(json);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  headServo.setPeriodHertz(50);
  headServo.attach(SERVO_PIN, 500, 2400);
  centerServo();
}

void loop() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "DIST") {
    sendDistance();
  } else if (command == "SCAN") {
    sendScan();
  } else if (command == "CENTER") {
    centerServo();
    Serial.println("{\"ok\":true,\"servo\":\"center\"}");
  } else {
    Serial.println("{\"ok\":false,\"error\":\"unknown command\"}");
  }
}
