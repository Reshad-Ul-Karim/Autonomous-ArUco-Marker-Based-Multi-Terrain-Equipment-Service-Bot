#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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
int gripMinUs  = 1300, gripMaxUs  = 1950;

// ---- Command ranges ----
int baseMinA  = 0,   baseMaxA  = 180;
int jointMinA = 20,  jointMaxA = 160;
// Gripper uses 0..100 (0=open, 100=close)
int gripMinA  = 30,   gripMaxA  = 100;

// ---- HOME pose (set your vertical straight here) ----
int HOME_BASE  = 90;
int HOME_JOINT = 90;  // change to your vertical straight value
int HOME_GRIP  = 0;

int baseAng = HOME_BASE, jointAng = HOME_JOINT, gripVal = HOME_GRIP;

uint16_t mapToCounts(int val, int minA, int maxA, int minUs, int maxUs) {
  val = constrain(val, minA, maxA);
  int us = map(val, minA, maxA, minUs, maxUs);
  return usToCounts(us);
}

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

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  goHome();
  Serial.println("READY");
  Serial.println("Send: base,joint,grip  e.g. 90,120,0 or 90,120,100");
  Serial.println("Or send: HOME");
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("HOME")) {
    goHome();
    Serial.println("OK HOME");
    return;
  }

  int b, j, g;
  int n = sscanf(line.c_str(), "%d,%d,%d", &b, &j, &g);
  if (n != 3) { Serial.println("ERR"); return; }

  baseAng  = constrain(b, baseMinA,  baseMaxA);
  jointAng = constrain(j, jointMinA, jointMaxA);
  gripVal  = constrain(g, gripMinA,  gripMaxA);

  writeServos();
  Serial.println("OK");
}
