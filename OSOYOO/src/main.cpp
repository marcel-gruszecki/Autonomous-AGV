// OSOYOO 2WD Robot Car - ESP32 WROOM
// Minimal TCP server used to bring up the motors and the line sensors.
//
// Wiring (see Scheme/Scheme.kicad_sch):
//   Motor driver (Model-X): IN1=GPIO23, IN2=GPIO22, IN3=GPIO19, IN4=GPIO18,
//                           ENA=GPIO17, ENB=GPIO16
//   Motor A -> terminal K1 (second lead to GND), controlled by IN1/IN2 + ENA
//   Motor B -> terminal K3 (second lead to GND), controlled by IN3/IN4 + ENB
//   K2 and K4 are unused.
//
//   Line sensors TCRT5000 (DO): left=GPIO34, centre=GPIO35, right=GPIO32
//
// NOTE: with K2/K4 left unconnected each motor is fed from a single half of the
// H-bridge and therefore spins IN ONE DIRECTION ONLY. A negative speed pulls the
// K1/K3 terminal to GND, i.e. the motor simply stops. Reverse motion requires
// wiring the second lead of each motor to K2/K4.
//
// SERIAL: 115200 baud (monitor_speed is set in platformio.ini).
//
// The board broadcasts its address over UDP port 8266 as "OSOYOO <ip> <port>",
// so the Rust client can locate the robot without reading the serial output.
//
// Protocol (one command per line, replies start with OK or ERR):
//   PING              -> OK PONG
//   M <left> <right>  -> OK M <l> <r>      speeds -255..255
//   S                 -> OK S              stop
//   SENS              -> OK SENS <l> <c> <r>   (1 = line / obstacle detected)
//   INFO              -> OK INFO <ip> <rssi> <uptime_ms>

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --------------------------------------------------------------- configuration
static const char *WIFI_SSID = "";   // <-- network name
static const char *WIFI_PASS = "";     // <-- password
static const uint16_t TCP_PORT = 8080;
static const uint16_t DISCOVERY_PORT = 8266;     // UDP broadcast carrying the IP
static const uint32_t BEACON_MS = 1000;

// Motor driver pins
static const int PIN_IN1 = 23;
static const int PIN_IN2 = 22;
static const int PIN_IN3 = 19;
static const int PIN_IN4 = 18;
static const int PIN_ENA = 17;
static const int PIN_ENB = 16;

// Sensor pins (GPIO34/35 are input-only; the TCRT5000 boards drive DO themselves)
static const int PIN_SENS_L = 34;
static const int PIN_SENS_M = 35;
static const int PIN_SENS_R = 32;

// PWM
static const int PWM_FREQ = 1000;
static const int PWM_BITS = 8;
static const int PWM_CH_A = 0;
static const int PWM_CH_B = 1;

// Safety: if the client goes quiet for longer than this, the motors are stopped.
static const uint32_t WATCHDOG_MS = 2000;

WiFiServer server(TCP_PORT);
WiFiClient client;
WiFiUDP udp;
uint32_t lastCommandMs = 0;
uint32_t lastBeaconMs = 0;
bool motorsRunning = false;

// ------------------------------------------------------------------- discovery
// Sends "OSOYOO <ip> <port>" to the subnet broadcast address so the client does
// not need to know the robot's IP.
static void sendBeacon() {
  IPAddress ip = WiFi.localIP();
  IPAddress mask = WiFi.subnetMask();
  IPAddress bcast(ip[0] | ~mask[0], ip[1] | ~mask[1], ip[2] | ~mask[2],
                  ip[3] | ~mask[3]);
  char msg[64];
  int n = snprintf(msg, sizeof(msg), "OSOYOO %s %u\n", ip.toString().c_str(),
                   TCP_PORT);
  udp.beginPacket(bcast, DISCOVERY_PORT);
  udp.write((const uint8_t *)msg, n);
  udp.endPacket();
}

// ---------------------------------------------------------------------- motors
static void pwmSetup(int pin, int channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, PWM_FREQ, PWM_BITS, channel);
#else
  ledcSetup(channel, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, channel);
#endif
}

static void pwmWrite(int pin, int channel, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

// speed: -255..255 (0 = stop).
// With the present wiring (K1 / K3 only) motion happens for speed > 0 only.
static void motorDrive(int inA, int inB, int enPin, int enCh, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else if (speed < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }
  pwmWrite(enPin, enCh, abs(speed));
}

static void setMotors(int left, int right) {
  motorDrive(PIN_IN1, PIN_IN2, PIN_ENA, PWM_CH_A, left);
  motorDrive(PIN_IN3, PIN_IN4, PIN_ENB, PWM_CH_B, right);
  motorsRunning = (left != 0 || right != 0);
}

static void stopMotors() { setMotors(0, 0); }

// -------------------------------------------------------------------- commands
static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String cmd = line;
  String args = "";
  int space = line.indexOf(' ');
  if (space >= 0) {
    cmd = line.substring(0, space);
    args = line.substring(space + 1);
    args.trim();
  }
  cmd.toUpperCase();

  if (cmd == "PING") {
    client.println("OK PONG");
  } else if (cmd == "S" || cmd == "STOP") {
    stopMotors();
    client.println("OK S");
  } else if (cmd == "M") {
    int sp = args.indexOf(' ');
    if (sp < 0) {
      client.println("ERR usage: M <left> <right>");
      return;
    }
    int left = args.substring(0, sp).toInt();
    int right = args.substring(sp + 1).toInt();
    setMotors(left, right);
    client.printf("OK M %d %d\n", left, right);
  } else if (cmd == "SENS") {
    client.printf("OK SENS %d %d %d\n", digitalRead(PIN_SENS_L),
                  digitalRead(PIN_SENS_M), digitalRead(PIN_SENS_R));
  } else if (cmd == "INFO") {
    client.printf("OK INFO %s %d %lu\n", WiFi.localIP().toString().c_str(),
                  WiFi.RSSI(), (unsigned long)millis());
  } else {
    client.println("ERR unknown command");
  }
}

// ----------------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pwmSetup(PIN_ENA, PWM_CH_A);
  pwmSetup(PIN_ENB, PWM_CH_B);
  stopMotors();

  pinMode(PIN_SENS_L, INPUT);
  pinMode(PIN_SENS_M, INPUT);
  pinMode(PIN_SENS_R, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to Wi-Fi \"%s\"", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.printf("\nConnected. IP: %s  port: %u\n",
                WiFi.localIP().toString().c_str(), TCP_PORT);

  server.begin();
  server.setNoDelay(true);
  udp.begin(DISCOVERY_PORT);
}

// ------------------------------------------------------------------------ loop
void loop() {
  if (millis() - lastBeaconMs > BEACON_MS) {
    lastBeaconMs = millis();
    if (WiFi.status() == WL_CONNECTED) sendBeacon();
  }

  if (!client || !client.connected()) {
    if (motorsRunning) stopMotors();
    WiFiClient incoming = server.available();
    if (incoming) {
      client = incoming;
      client.setNoDelay(true);
      lastCommandMs = millis();
      client.println("OK HELLO osoyoo-esp32");
      Serial.println("Client connected.");
    }
    return;
  }

  static String buffer;
  while (client.available()) {
    char c = (char)client.read();
    if (c == '\n') {
      lastCommandMs = millis();
      handleCommand(buffer);
      buffer = "";
    } else if (c != '\r' && buffer.length() < 64) {
      buffer += c;
    }
  }

  // Watchdog: no commands for a while -> stop the motors.
  if (motorsRunning && millis() - lastCommandMs > WATCHDOG_MS) {
    stopMotors();
    client.println("OK S watchdog");
  }
}
