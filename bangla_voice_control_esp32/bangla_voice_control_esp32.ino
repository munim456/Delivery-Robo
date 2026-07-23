/*
  Bangla Voice-Controlled RC Car — Follow Me + Lid Servo + AI Chat
  Board: ESP32-S3-WROOM-1 N16R8 (Arduino IDE)
  COMMS: WiFi (HTTP) — phone and ESP32 must be on the SAME Wi-Fi network.

  ARCHITECTURE:
  The ESP32 does NOT talk to any AI itself. Your phone (using the
  companion "bangla_voice_control_wifi.html" page) listens in Bangla,
  has a conversation with Gemini (Google AI Studio) to figure out what
  you want, and only once it's decided on a specific action, sends that
  ONE canonical Bangla command word to this sketch as a plain HTTP GET
  request over your local Wi-Fi. This sketch drives the hardware and
  sends back a short Bangla reply as the HTTP response body, which your
  phone speaks aloud.

  WHAT CHANGED FROM YOUR ORIGINAL SKETCH:
   - Added RFID "card just scanned" tracking + a new /rfid_status
     endpoint, so the phone can poll and say "ধন্যবাদ" (thank you)
     when you tap a card. Everything else is unchanged.

  RFID WIRING REMINDER (per your note):
   - The MFRC522 module is 3.3V ONLY, NOT 5V-tolerant.
   - Power VCC from the ESP32's 3V3 pin, never from a 5V rail.

  REQUIRED LIBRARIES (Arduino IDE > Tools > Manage Libraries):
   - ESP32Servo
   - Adafruit GFX Library
   - Adafruit SSD1306
   - Adafruit Unified Sensor
   - Adafruit BMP085 Library      (works for BMP180 / GY-68)
   - DHT sensor library (by Adafruit)
   - MFRC522                     (by GithubCommunity)
   (WiFi.h, WebServer.h, ESPmDNS.h come built-in with the ESP32 board
    package.)

  BOARD SETTINGS (Tools menu):
   - Board: "ESP32S3 Dev Module"
   - USB CDC On Boot: "Enabled"
   - Flash Size: "16MB"
   - PSRAM: "OPI PSRAM"
   - Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)" or similar

  BEFORE UPLOADING:
   - Set WIFI_SSID / WIFI_PASSWORD below to your home Wi-Fi.
   - After upload, open Serial Monitor at 115200 baud — the car
     will print its IP address once connected. Type that IP into
     the HTML page on your phone.

  CALIBRATE before first real use:
   - LID_OPEN_ANGLE / LID_CLOSED_ANGLE for your box
   - FOLLOW_FAR_CM / FOLLOW_NEAR_CM for your follow-me comfort distance
*/

#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ---------------- WiFi settings ----------------
const char* WIFI_SSID     = "Mozahar2";
const char* WIFI_PASSWORD = "mozahar3118";

// mDNS hostname — lets you optionally use http://banglarc.local/ instead
// of typing the IP. Works on many phones, but not guaranteed on all
// browsers, so the IP address (printed on Serial + shown on OLED) is
// still the reliable fallback.
const char* MDNS_NAME = "banglarc";

// ---------------- Pin map (unchanged) ----------------
#define I2C_SDA   1
#define I2C_SCL   2

#define L_RPWM    4
#define L_LPWM    5
#define L_EN      6

#define R_RPWM    7
#define R_LPWM    8
#define R_EN      9

#define DHT_PIN   11
#define DHT_TYPE  DHT11

#define SERVO_PIN 21

#define TRIG_PIN  12
#define ECHO_PIN  13

#define RFID_SCK  39
#define RFID_MISO 41
#define RFID_MOSI 40
#define RFID_SS   42
#define RFID_RST  38

// ---------------- Objects ----------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool bmpOK = false;
Adafruit_BMP085 bmp;
DHT dht(DHT_PIN, DHT_TYPE);
Servo lidServo;
MFRC522 rfid(RFID_SS, RFID_RST);

// ---------------- WiFi / HTTP server ----------------
WebServer server(80);
bool wifiConnected = false;
String replyText = "-"; // last Bangla reply, set by speak()

// ---------------- State ----------------
enum Mode { IDLE, DRIVING, FOLLOWING };
Mode currentMode = IDLE;
unsigned long driveUntil = 0;

const int DRIVE_MS    = 1000;  // each voice drive command runs this long
const int DRIVE_SPEED = 200;   // 0-255
const int TURN_SPEED  = 180;

const float FOLLOW_FAR_CM  = 40; // if farther than this, drive forward
const float FOLLOW_NEAR_CM = 20; // if closer than this, stop

int LID_OPEN_ANGLE   = 90;  // calibrate for your box
int LID_CLOSED_ANGLE = 0;   // calibrate for your box
bool lidOpen = false;

unsigned long lastSensorRead = 0;
float temperature = 0, distanceCm = -1;
double pressure = 0;
String lastCmdShown = "-";

// ---------------- RFID scan tracking (NEW) ----------------
bool cardJustScanned = false;      // set true in loop(), cleared once the phone polls it
String lastCardUidHex = "";        // last UID read, as hex text
unsigned long lastCardMillis = 0;  // for a simple re-trigger cooldown
const unsigned long CARD_COOLDOWN_MS = 3000; // ignore the same card sitting on the reader

// ---------------- Reply helper ----------------
void speak(const String &bnText) {
  replyText = bnText;
  Serial.println(bnText);
}

// ---------------- Motor control (unchanged) ----------------
void setLeftMotor(int speed) {
  if (speed >= 0) { analogWrite(L_RPWM, speed); analogWrite(L_LPWM, 0); }
  else            { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, -speed); }
}
void setRightMotor(int speed) {
  if (speed >= 0) { analogWrite(R_RPWM, speed); analogWrite(R_LPWM, 0); }
  else            { analogWrite(R_RPWM, 0); analogWrite(R_LPWM, -speed); }
}
void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  currentMode = IDLE;
}
void turnLeft(int spd = DRIVE_SPEED) { setLeftMotor(spd); setRightMotor(spd); }
void turnRight(int spd = DRIVE_SPEED) { setLeftMotor(-spd); setRightMotor(-spd); }
void driveForward(int spd = TURN_SPEED)  { setLeftMotor(-spd); setRightMotor(spd); }
void driveBackward(int spd = TURN_SPEED) { setLeftMotor(spd);  setRightMotor(-spd); }

// ---------------- Ultrasonic (unchanged) ----------------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout ~ 4m
  if (duration == 0) return -1;
  return duration * 0.0343f / 2.0f;
}

// ---------------- Lid servo (unchanged) ----------------
void openLid() {
  lidServo.write(LID_OPEN_ANGLE);
  lidOpen = true;
  speak("বক্স খোলা হয়েছে");
}
void closeLid() {
  lidServo.write(LID_CLOSED_ANGLE);
  lidOpen = false;
  speak("বক্স বন্ধ করা হয়েছে");
}

// ---------------- OLED ----------------
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("RC Car Status");
  display.print("Mode: ");
  display.println(currentMode == IDLE ? "Idle" : currentMode == DRIVING ? "Drive" : "Follow");
  display.print("Lid: ");  display.println(lidOpen ? "Open" : "Closed");
  display.print("Dist: "); display.print(distanceCm, 0); display.println("cm");
  display.print("Temp: "); display.print(temperature, 1); display.println("C");
  display.print("Cmd: ");  display.println(lastCmdShown);
  if (wifiConnected) {
    display.println(WiFi.localIP().toString());
  } else {
    display.println("WiFi: connecting...");
  }
  display.display();
}

// ---------------- Command handling ----------------
// Called from the HTTP handler with one of the canonical Bangla
// words/phrases. Returns the Bangla reply text (also stored via speak()).
String handleCommand(String cmd) {
  lastCmdShown = cmd;
  Serial.print("Command = ");
  Serial.println(cmd);

  if (cmd == "সামনে") {
    currentMode = DRIVING;
    driveForward();
    driveUntil = millis() + DRIVE_MS;
    speak("সামনে যাচ্ছি");
  } else if (cmd == "পেছনে") {
    currentMode = DRIVING;
    driveBackward();
    driveUntil = millis() + DRIVE_MS;
    speak("পেছনে যাচ্ছি");
  } else if (cmd == "বামে") {
    currentMode = DRIVING;
    turnLeft();
    driveUntil = millis() + DRIVE_MS;
    speak("বামে ঘুরছি");
  } else if (cmd == "ডানে") {
    currentMode = DRIVING;
    turnRight();
    driveUntil = millis() + DRIVE_MS;
    speak("ডানে ঘুরছি");
  } else if (cmd == "থামো") {
    stopMotors();
    speak("থেমে গেছি");
  } else if (cmd == "ফলো") {
    currentMode = FOLLOWING;
    speak("আমি আপনাকে ফলো করছি");
  } else if (cmd == "ফলো বন্ধ") {
    stopMotors();
    speak("ফলো করা বন্ধ করেছি");
  } else if (cmd == "খোলো") {
    openLid();
  } else if (cmd == "বন্ধ") {
    closeLid();
  } else if (cmd == "তাপমাত্রা") {
    String msg = "তাপমাত্রা " + String(temperature, 1) +
                 " ডিগ্রি, দূরত্ব " + String(distanceCm, 0) + " সেন্টিমিটার";
    speak(msg);
  } else {
    speak("দুঃখিত, বুঝতে পারিনি");
  }

  updateDisplay();
  return replyText;
}

// ---------------- HTTP handlers ----------------
// GET /cmd?c=<command>  -> runs the command, replies with Bangla text
void handleCmdRequest() {
  server.sendHeader("Access-Control-Allow-Origin", "*"); // allow phone page to fetch
  if (!server.hasArg("c")) {
    server.send(400, "text/plain; charset=utf-8", "কমান্ড পাইনি");
    return;
  }
  String cmd = server.arg("c");
  String reply = handleCommand(cmd);
  server.send(200, "text/plain; charset=utf-8", reply);
}

// GET /rfid_status -> {"scanned":true/false,"msg":"...","uid":"..."}
// The phone polls this every couple of seconds. If a card was tapped
// since the last poll, "scanned" is true ONCE, then clears itself, so
// the phone only says "thank you" a single time per tap.
void handleRfidStatus() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String json = "{";
  if (cardJustScanned) {
    json += "\"scanned\":true,";
    json += "\"msg\":\"ধন্যবাদ\",";
    json += "\"uid\":\"" + lastCardUidHex + "\"";
    cardJustScanned = false; // consumed — won't fire again until next tap
  } else {
    json += "\"scanned\":false,";
    json += "\"msg\":\"\",";
    json += "\"uid\":\"\"";
  }
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

// GET /  -> simple health check page, useful for testing in a browser
void handleRoot() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String msg = "BanglaRC-Car is online. IP: " + WiFi.localIP().toString();
  server.send(200, "text/plain; charset=utf-8", msg);
}

void handleNotFound() {
  Serial.println("404 - path was: " + server.uri() + " args: " + server.args());  // ADD THIS LINE
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(404, "text/plain", "Not found");
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("==========================");
  Serial.println("SETUP STARTED");
  Serial.println("==========================");

  Serial.println("1");
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT); pinMode(L_EN, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT); pinMode(R_EN, OUTPUT);

  Serial.println("2");
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);

  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);

  Serial.println("3");
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("4");
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("5");
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("OLED FAILED");
    while(true);
  }
  Serial.println("OLED OK");

  display.clearDisplay();
  display.display();

  Serial.println("6");
  bmpOK = bmp.begin();

  if (bmpOK) {
    Serial.println("BMP OK");
  } else {
    Serial.println("BMP FAILED");
  }

  Serial.println("7");
  dht.begin();

  Serial.println("8");
  // lidServo.setPeriodHertz(50);
  // lidServo.attach(SERVO_PIN, 500, 2400);

  Serial.println("9");
  closeLid();

  Serial.println("10");
  // RFID: MFRC522 is 3.3V only — VCC must come from the ESP32's 3V3 pin.
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);

  Serial.println("11");
  rfid.PCD_Init();

  Serial.println("12");
  WiFi.mode(WIFI_STA);

  Serial.println("13");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("14");
  Serial.println("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("15");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/cmd", handleCmdRequest);
  server.on("/rfid_status", handleRfidStatus);
  server.begin();

  Serial.println("16");
}

// ---------------- Loop ----------------
void loop() {
  Serial.println("Loop started");
  delay(1000);
  server.handleClient();

  // Keep wifiConnected flag accurate + auto-retry if the connection drops
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      wifiConnected = false;
      Serial.println("WiFi disconnected, retrying...");
    }
    WiFi.reconnect();
  } else if (!wifiConnected) {
    wifiConnected = true;
    Serial.print("WiFi reconnected. IP address: ");
    Serial.println(WiFi.localIP());
  }

  // Auto-stop after a timed drive pulse
  if (currentMode == DRIVING && millis() > driveUntil) {
    stopMotors();
  }

  // Follow-me behavior
  if (currentMode == FOLLOWING) {
    if (distanceCm < 0) {
      setLeftMotor(0); setRightMotor(0);
    } else if (distanceCm > FOLLOW_FAR_CM) {
      driveForward(DRIVE_SPEED);
    } else if (distanceCm < FOLLOW_NEAR_CM) {
      setLeftMotor(0); setRightMotor(0);
    } else {
      driveForward(120);
    }
  }

  // Periodic sensor read
  if (millis() - lastSensorRead > 300) {
    lastSensorRead = millis();
    distanceCm = readDistanceCm();
    temperature = dht.readTemperature();
    if (bmpOK) {
      pressure = bmp.readPressure();
    } else {
      pressure = 0;
    }
    updateDisplay();
  }

  // RFID: when a new card is tapped, flag it so the phone can say "thank you"
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    String uidHex = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] < 0x10) uidHex += "0";
      uidHex += String(rfid.uid.uidByte[i], HEX);
    }
    Serial.print("Card UID: ");
    Serial.println(uidHex);

    if (millis() - lastCardMillis > CARD_COOLDOWN_MS) {
      cardJustScanned = true;
      lastCardUidHex = uidHex;
      lastCardMillis = millis();
      speak("ধন্যবাদ"); // also reflected on OLED / Serial
      updateDisplay();
    }

    rfid.PICC_HaltA();
  }
}
