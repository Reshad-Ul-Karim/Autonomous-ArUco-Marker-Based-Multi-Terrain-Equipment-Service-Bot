#include <Arduino.h>
#include <WiFi.h>

// ====== Hotspot credentials ======
const char* WIFI_SSID = "*****";
const char* WIFI_PASS = "*******";

// Give each ESP a unique name later: SlayerX-01, SlayerX-02, ...
const char* DEVICE_NAME = "SlayerX-01";

// TCP server
WiFiServer server(3333);
WiFiClient client;

// Incoming command
char BT = 0;
int Speed = 100;

//Right
int R1PWM = 19;
int R2PWM = 21;
//Left
int L1PWM = 23;
int L2PWM = 22;

#define rmf 0
#define rmb 1
#define lmf 2
#define lmb 3

// ---- Failsafe ----
unsigned long lastCmdMs = 0;
const unsigned long CMD_TIMEOUT_MS = 400;

// ===== Motor functions =====
void go_forward();
void go_backward();
void go_left();
void go_right();
void stopBot();
void forward_right();
void backward_right();
void forward_left();
void backward_left();

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);            // IMPORTANT: reduces latency + dropouts
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
  Serial.println("TCP port: 3333");
}

void handleByte(char c) {
  lastCmdMs = millis();
  Serial.println(c);

  // speed control
  if (c == 'x') Speed = 0;       // TRUE zero speed
  else if (c == '0') Speed = 100;
  else if (c == '1') Speed = 110;
  else if (c == '2') Speed = 120;
  else if (c == '3') Speed = 130;
  else if (c == '4') Speed = 140;
  else if (c == '5') Speed = 150;
  else if (c == '6') Speed = 180;
  else if (c == '7') Speed = 200;
  else if (c == '8') Speed = 220;
  else if (c == '9') Speed = 240;
  else if (c == 'q') Speed = 255;

  // movement
  if (c == 'F') go_forward();
  else if (c == 'B') go_backward();
  else if (c == 'L') go_left();
  else if (c == 'R') go_right();
  else if (c == 'S') stopBot();
  else if (c == 'I') forward_right();
  else if (c == 'J') backward_right();
  else if (c == 'G') forward_left();
  else if (c == 'H') backward_left();
}

void setup() {
  Serial.begin(115200);

  pinMode(R1PWM, OUTPUT);
  pinMode(R2PWM, OUTPUT);
  pinMode(L1PWM, OUTPUT);
  pinMode(L2PWM, OUTPUT);

  // Setup PWM channels
  ledcSetup(rmf, 5000, 8);
  ledcAttachPin(R1PWM, rmf);

  ledcSetup(rmb, 5000, 8);
  ledcAttachPin(R2PWM, rmb);

  ledcSetup(lmf, 5000, 8);
  ledcAttachPin(L1PWM, lmf);

  ledcSetup(lmb, 5000, 8);
  ledcAttachPin(L2PWM, lmb);

  stopBot();
  lastCmdMs = millis();

  connectWiFi();
  server.begin();
  server.setNoDelay(true);

  Serial.println("✅ Wi-Fi control ready.");
}

void loop() {
  // If Wi-Fi drops, stop and reconnect
  if (WiFi.status() != WL_CONNECTED) {
    stopBot();
    connectWiFi();
  }

  // Accept a client (one controller at a time)
  if (!client || !client.connected()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      client.stop();     // ensure only 1 client
      client = newClient;
      client.setNoDelay(true);
      Serial.println("✅ Controller connected.");
      lastCmdMs = millis();
    }
  }

  // Read bytes
  if (client && client.connected()) {
    while (client.available()) {
      BT = (char)client.read();
      handleByte(BT);
    }
  } else {
    // If no client connected, keep stopped
    stopBot();
  }

  // Safety timeout: if no command recently, stop
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    stopBot();
  }
}

// ---- Motor functions ----
void go_forward() {
  ledcWrite(rmf, Speed);  ledcWrite(rmb, 0);
  ledcWrite(lmf, Speed);  ledcWrite(lmb, 0);
}

void go_backward() {
  ledcWrite(rmf, 0);      ledcWrite(rmb, Speed);
  ledcWrite(lmf, 0);      ledcWrite(lmb, Speed);
}

void go_left() {
  ledcWrite(rmf, 0);      ledcWrite(rmb, Speed);
  ledcWrite(lmf, Speed);  ledcWrite(lmb, 0);
}

void go_right() {
  ledcWrite(rmf, Speed);  ledcWrite(rmb, 0);
  ledcWrite(lmf, 0);      ledcWrite(lmb, Speed);
}

void stopBot() {
  ledcWrite(rmf, 0); ledcWrite(rmb, 0);
  ledcWrite(lmf, 0); ledcWrite(lmb, 0);
}

void forward_right() {
  ledcWrite(rmf, Speed);  ledcWrite(rmb, 0);
  ledcWrite(lmf, 0);      ledcWrite(lmb, 0);
}

void backward_right() {
  ledcWrite(rmf, 0);      ledcWrite(rmb, Speed);
  ledcWrite(lmf, 0);      ledcWrite(lmb, 0);
}

void forward_left() {
  ledcWrite(rmf, 0);      ledcWrite(rmb, 0);
  ledcWrite(lmf, Speed);  ledcWrite(lmb, 0);
}

void backward_left() {
  ledcWrite(rmf, 0);      ledcWrite(rmb, 0);
  ledcWrite(lmf, 0);      ledcWrite(lmb, Speed);
}
