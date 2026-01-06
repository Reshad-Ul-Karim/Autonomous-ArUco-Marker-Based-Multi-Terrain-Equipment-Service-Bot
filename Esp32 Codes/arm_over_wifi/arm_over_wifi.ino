#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ================= Wi-Fi =================
const char* WIFI_SSID = "*******";
const char* WIFI_PASS = "*******";
const char* DEVICE_NAME = "SlayerX-ARM-01";

// Use a DIFFERENT port than the drive ESP (drive uses 3333)
WiFiServer server(3334);
WiFiClient client;

// ---- Failsafe ----
unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 800;   // arm timeout

// ================= PCA9685 =================
Adafruit_PWMServoDriver pwm(0x40);

// PCA9685 channels
#define BASE_SERVO  0
#define JOINT_SERVO 1
#define GRIP_SERVO  3

// PCA9685 timing
const float PWM_FREQ  = 50.0f;
const float PERIOD_US = 1000000.0f / PWM_FREQ;

uint16_t usToCounts(int us) {
  float counts = (us * 4096.0f) / PERIOD_US;
  if (counts < 0) counts = 0;
  if (counts > 4095) counts = 4095;
  return (uint16_t)counts;
}

// ---- Microsecond ranges (from your working sweep) ----
int baseMinUs  = 700,  baseMaxUs  = 2300;
int jointMinUs = 800,  jointMaxUs = 2200;
int gripMinUs  = 1400, gripMaxUs  = 1950;

// ---- Command ranges ----
int baseMinA  = 0,   baseMaxA  = 180;
int jointMinA = 20,  jointMaxA = 160;
// Gripper uses 0..100 (0=open, 100=close)
int gripMinA  = 60,  gripMaxA  = 100;

// ---- HOME pose ----
int HOME_BASE  = 90;
int HOME_JOINT = 90;
int HOME_GRIP  = 0;

int baseAng = HOME_BASE, jointAng = HOME_JOINT, gripVal = HOME_GRIP;

uint16_t mapToCounts(int val, int minA, int maxA, int minUs, int maxUs) {
  val = constrain(val, minA, maxA);
  int us = map(val, minA, maxA, minUs, maxUs);
  return usToCounts(us);
}
//print
void writeServos() {
  pwm.setPWM(BASE_SERVO,  0, mapToCounts(baseAng,  baseMinA,  baseMaxA,  baseMinUs,  baseMaxUs));
  pwm.setPWM(JOINT_SERVO, 0, mapToCounts(jointAng, jointMinA, jointMaxA, jointMinUs, jointMaxUs));
  pwm.setPWM(GRIP_SERVO,  0, mapToCounts(gripVal,  gripMinA,  gripMaxA,  gripMinUs,  gripMaxUs));
}

void goHome() {
  baseAng  = HOME_BASE;
  jointAng = HOME_JOINT;
  gripVal  = HOME_GRIP;
  writeServos();
}

// ================= Wi-Fi helpers =================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(DEVICE_NAME);

  Serial.printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - start > 20000) {
      Serial.println("\nRetrying Wi-Fi...");
      WiFi.disconnect(true);
      delay(300);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      start = millis();
    }
  }

  Serial.println("\n✅ Wi-Fi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.println("TCP port: 3334");
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  lastCmdMs = millis();

  if (line.equalsIgnoreCase("HOME")) {
    goHome();
    client.println("OK HOME");
    return;
  }

  int b, j, g;
  int n = sscanf(line.c_str(), "%d,%d,%d", &b, &j, &g);
  if (n != 3) {
    client.println("ERR");
    return;
  }

  baseAng  = constrain(b, baseMinA,  baseMaxA);
  jointAng = constrain(j, jointMinA, jointMaxA);
  gripVal  = constrain(g, gripMinA,  gripMaxA);

  writeServos();
  client.println("OK");
}

// Simple line reader from TCP (collects until '\n')
String rxLine;

void setup() {
  Serial.begin(115200);

  // I2C pins for ESP32 -> PCA9685 (you used 21,22)
  Wire.begin(21, 22);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  goHome();

  connectWiFi();
  server.begin();
  server.setNoDelay(true);

  Serial.println("✅ Arm Wi-Fi control ready.");
  Serial.println("Send over TCP: HOME or base,joint,grip (e.g. 90,120,0)");
}

void loop() {
  // If Wi-Fi drops, go safe and reconnect
  if (WiFi.status() != WL_CONNECTED) {
    goHome();
    connectWiFi();
  }

  // Accept single client
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client.stop();
      client = newClient;
      client.setNoDelay(true);
      rxLine = "";
      lastCmdMs = millis();
      Serial.println("✅ Arm controller connected.");
      client.println("READY");
    } else {
      // no client -> keep home (or keep last pose if you prefer)
      // goHome();
    }
  }

  // Read bytes and build lines
  if (client && client.connected()) {
    while (client.available()) {
      char c = (char)client.read();
      if (c == '\r') continue;

      if (c == '\n') {
        handleLine(rxLine);
        rxLine = "";
      } else {
        // prevent runaway memory
        if (rxLine.length() < 80) rxLine += c;
        else rxLine = "";
      }
    }
  }

  // Failsafe: if no command recently -> HOME
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    goHome();
    lastCmdMs = millis(); // prevent spamming HOME continuously
  }
}
