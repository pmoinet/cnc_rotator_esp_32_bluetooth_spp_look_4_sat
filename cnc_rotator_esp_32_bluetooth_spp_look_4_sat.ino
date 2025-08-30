/*
  Satnogs-CNC-Rotator — ESPduino-32 (Wemos D1 R32) + BT SPP + USB Serial + WiFi WebUI

  Features added in this file compared to the earlier demo:
   - Pin mapping for ESPduino-32 / CNC Shield v3
   - X and Y endstop handling (min endstops)
   - Homing command ('H') usable from Serial / Bluetooth / Web
   - WiFi: attempts STA with hardcoded credentials, falls back to AP
   - Simple Web server UI to manually control & monitor rotator
   - All transports share the same command parser (GS-232/Easycomm-like)

  Hardware notes
  - ESPduino-32 (Wemos D1 R32) pins follow mapping below.
  - CNC Shield v3 drivers (A4988/DRV8825) step/dir pins wired accordingly.
  - Endstops wired to X_MIN_PIN and Y_MIN_PIN, using INPUT_PULLUP.

  Edit at top to change WiFi credentials, mechanical calibration, and pins.
*/

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <WebServer.h>

// -------------------- User configuration --------------------
// Bluetooth device name
static const char* BT_NAME = "SatnogsRotator";

// WiFi STA credentials (edit) — leave empty ("") to skip STA and start AP
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASS";

// Access Point fallback (if STA fails)
const char* AP_SSID = "SatnogsRotator_AP";
const char* AP_PASS = "rotator123";

// Steps per degree (calibrate to your mechanics)
float AZ_STEPS_PER_DEG = 8.8889f;
float EL_STEPS_PER_DEG = 8.8889f;

// Speed/accel (steps/s, steps/s^2)
const float MAX_SPEED_STEPS_S = 1200.0f;
const float ACCEL_STEPS_S2    = 800.0f;

// Soft limits (degrees)
const float AZ_MIN_DEG =   0.0f;
const float AZ_MAX_DEG = 360.0f;
const float EL_MIN_DEG =   0.0f;
const float EL_MAX_DEG = 180.0f;

// -------------------- Pin mapping (ESPduino-32 / CNC Shield) --------------------
// Azimuth (X)
const int AZ_STEP_PIN = 2;   // D2
const int AZ_DIR_PIN  = 5;   // D5

// Elevation (Y)
const int EL_STEP_PIN = 4;   // D3 -> GPIO4 mapping on this board
const int EL_DIR_PIN  = 16;  // D6 -> GPIO16

// Enable common
const int DRV_EN_PIN  = 15;  // D8 -> GPIO15
bool     DRV_ENABLE_ACTIVE_LOW = true;

// Endstops (min)
const int X_MIN_PIN = 17; // D9 -> GPIO17
const int Y_MIN_PIN = 18; // D10 -> GPIO18

// --------------------------------------------------------------

AccelStepper azStepper(AccelStepper::DRIVER, AZ_STEP_PIN, AZ_DIR_PIN);
AccelStepper elStepper(AccelStepper::DRIVER, EL_STEP_PIN, EL_DIR_PIN);

BluetoothSerial SerialBT;
WebServer server(80);

// Streams sinks: USB Serial and BT (we don't place WebServer as Stream)
Stream* sinks[2];
size_t  sinkCount = 0;

volatile bool stopRequested = false;

// Utilities
long degToStepsAZ(float deg){ return lroundf(deg * AZ_STEPS_PER_DEG); }
long degToStepsEL(float deg){ return lroundf(deg * EL_STEPS_PER_DEG); }
float stepsToDegAZ(long st){ return (float)st / AZ_STEPS_PER_DEG; }
float stepsToDegEL(long st){ return (float)st / EL_STEPS_PER_DEG; }
float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }

void motorEnable(bool en){
  if (DRV_EN_PIN < 0) return;
  digitalWrite(DRV_EN_PIN, DRV_ENABLE_ACTIVE_LOW ? !en : en);
}

void setupMotors(){
  azStepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  azStepper.setAcceleration(ACCEL_STEPS_S2);
  elStepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  elStepper.setAcceleration(ACCEL_STEPS_S2);
  if (DRV_EN_PIN >= 0) pinMode(DRV_EN_PIN, OUTPUT);
  motorEnable(true);
}

void sendAll(const String& s){ for(size_t i=0;i<sinkCount;i++) sinks[i]->print(s); }
void sendAllLn(const String& s){ for(size_t i=0;i<sinkCount;i++) sinks[i]->println(s); }

// -------------------- Command parser (Serial/BT/Web gateways use it) --------------------
// Supported commands (case-insensitive):
// - "AZxxx.x ELyyy.y" -> move absolute
// - "C" -> report current position: "AZ=xxx.x EL=yyy.y"
// - "S" -> stop
// - "R" -> reset current position to 0
// - "H" -> home (blocks until both axes hit min endstops)

void doHoming(){
  // homing strategy: move negative (toward min endstops) until switches close
  sendAllLn("HOMING START");

  // AZ homing
  azStepper.setMaxSpeed(300);
  azStepper.moveTo(degToStepsAZ(-360.0)); // large negative move
  while (digitalRead(X_MIN_PIN) == HIGH){ // HIGH = not triggered (INPUT_PULLUP)
    azStepper.run();
  }
  azStepper.setCurrentPosition(0);
  azStepper.stop();

  // EL homing
  elStepper.setMaxSpeed(300);
  elStepper.moveTo(degToStepsEL(-360.0));
  while (digitalRead(Y_MIN_PIN) == HIGH){
    elStepper.run();
  }
  elStepper.setCurrentPosition(0);
  elStepper.stop();

  // restore speeds
  azStepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  elStepper.setMaxSpeed(MAX_SPEED_STEPS_S);

  sendAllLn("HOMING DONE");
}

void handleLine(const String& line){
  String s = line; s.trim();
  if (s.length() == 0) return;
  if (s.equalsIgnoreCase("C")){
    float az = stepsToDegAZ(azStepper.currentPosition());
    float el = stepsToDegEL(elStepper.currentPosition());
    char buf[64];
    snprintf(buf, sizeof(buf), "AZ=%.1f EL=%.1f", az, el);
    sendAllLn(String(buf));
    return;
  }
  if (s.equalsIgnoreCase("S")){
    stopRequested = true;
    azStepper.stop(); elStepper.stop();
    sendAllLn("OK");
    return;
  }
  if (s.equalsIgnoreCase("R")){
    azStepper.setCurrentPosition(0); elStepper.setCurrentPosition(0);
    sendAllLn("OK");
    return;
  }
  if (s.equalsIgnoreCase("H")){
    doHoming();
    return;
  }

  // Set command parsing
  float azDeg = NAN, elDeg = NAN;
  int from = 0;
  while (from < s.length()){
    int sp = s.indexOf(' ', from);
    if (sp < 0) sp = s.length();
    String tok = s.substring(from, sp); tok.trim();
    if (tok.length() >= 3){
      String key = tok.substring(0,2);
      String val = tok.substring(2);
      if (key.equalsIgnoreCase("AZ")) azDeg = val.toFloat();
      if (key.equalsIgnoreCase("EL")) elDeg = val.toFloat();
    }
    from = sp + 1;
  }

  bool any = false;
  if (!isnan(azDeg)){
    azDeg = clampf(azDeg, AZ_MIN_DEG, AZ_MAX_DEG);
    azStepper.moveTo(degToStepsAZ(azDeg)); any = true;
  }
  if (!isnan(elDeg)){
    elDeg = clampf(elDeg, EL_MIN_DEG, EL_MAX_DEG);
    elStepper.moveTo(degToStepsEL(elDeg)); any = true;
  }
  if (any){ stopRequested = false; sendAllLn("OK"); } else { sendAllLn("ERR"); }
}

// Stream processing: simple line buffer per Stream
void processStream(Stream& io){
  static String bufUSB, bufBT;
  String &buf = (&io == &Serial) ? bufUSB : bufBT;

  while (io.available()){
    char c = (char)io.read();
if (c == '\n') {
  handleLine(buf);
  buf = "";
}else {
      if (buf.length() < 128) buf += c;
    }
  }
}

// -------------------- Web UI --------------------
const char* PAGE = R"=====(
<!doctype html>
<html>
<head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Satnogs Rotator</title></head>
<body>
<h2>Satnogs Rotator</h2>
<div>
  <label>AZ: <input id="az" type="number" step="0.1" value="0"></label>
  <label>EL: <input id="el" type="number" step="0.1" value="0"></label>
  <button onclick="move()">Move</button>
  <button onclick="home()">Homing</button>
  <button onclick="stop()">Stop</button>
  <button onclick="query()">Status</button>
</div>
<pre id="out"></pre>
<script>
function ajax(path){ fetch(path).then(r=>r.text()).then(t=>document.getElementById('out').innerText=t); }
function move(){ let az=document.getElementById('az').value; let el=document.getElementById('el').value; ajax('/move?az='+az+'&el='+el); }
function home(){ ajax('/home'); }
function stop(){ ajax('/stop'); }
function query(){ ajax('/status'); }
setInterval(query,3000);
</script>
</body></html>
)=====";

void handleRoot(){ server.send(200, "text/html", PAGE); }

void handleMove(){
  String az = server.arg("az");
  String el = server.arg("el");
  String cmd = "";
  if (az.length()) cmd += String("AZ") + az + " ";
  if (el.length()) cmd += String("EL") + el;
  handleLine(cmd);
  server.send(200, "text/plain", "OK");
}

void handleHome(){ handleLine("H"); server.send(200, "text/plain", "HOMING"); }
void handleStop(){ handleLine("S"); server.send(200, "text/plain", "STOPPED"); }

void handleStatus(){
  char buf[80];
  float az = stepsToDegAZ(azStepper.currentPosition());
  float el = stepsToDegEL(elStepper.currentPosition());
  int xm = digitalRead(X_MIN_PIN);
  int ym = digitalRead(Y_MIN_PIN);
  snprintf(buf, sizeof(buf), "AZ=%.1f EL=%.1f, X_min=%d Y_min=%d", az, el, xm, ym);
  server.send(200, "text/plain", String(buf));
}

// -------------------- Setup & Loop --------------------

void setup(){
  Serial.begin(115200);
  delay(100);
  Serial.println("Satnogs-CNC-Rotator (ESPduino-32) starting...");

  // Pins
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(DRV_EN_PIN, OUTPUT);

  // Motors
  setupMotors();
  azStepper.setCurrentPosition(0);
  elStepper.setCurrentPosition(0);

  // Bluetooth
  if (!SerialBT.begin(BT_NAME)){
    Serial.println("[BT] Init failed!");
  } else {
    Serial.print("[BT] Ready as: "); Serial.println(BT_NAME);
  }

  // register sinks (USB + BT)
  sinkCount = 0; sinks[sinkCount++] = &Serial; sinks[sinkCount++] = &SerialBT;

  // WiFi: try STA then fallback to AP
  bool wifiConnected = false;
  if (strlen(WIFI_SSID) > 0){
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi...");
    unsigned long start = millis();
    while (millis() - start < 8000){
      if (WiFi.status() == WL_CONNECTED){ wifiConnected = true; break; }
      delay(250); Serial.print('.');
    }
    Serial.println();
  }
  if (wifiConnected){
    Serial.print("WiFi STA connected: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("Starting AP mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  }

  // Web server endpoints
  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.on("/home", handleHome);
  server.on("/stop", handleStop);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("Web server started");

  sendAllLn("READY");
}

unsigned long lastEndstopCheck = 0;

void loop(){
  // Serial/Bluetooth inputs
  processStream(Serial);
  processStream(SerialBT);

  // Web server
  server.handleClient();

  // Run steppers (non-blocking run)
  if (!stopRequested){ azStepper.run(); elStepper.run(); } else { azStepper.run(); elStepper.run(); }

  // Poll endstops regularly and act
  if (millis() - lastEndstopCheck > 50){
    lastEndstopCheck = millis();
    if (digitalRead(X_MIN_PIN) == LOW){ // triggered (INPUT_PULLUP)
      azStepper.stop(); azStepper.setCurrentPosition(0); sendAllLn("X_MIN_TRIG");
    }
    if (digitalRead(Y_MIN_PIN) == LOW){
      elStepper.stop(); elStepper.setCurrentPosition(0); sendAllLn("Y_MIN_TRIG");
    }
  }
}

/*
Notes and next steps
- Edit WIFI_SSID / WIFI_PASS at top, or leave empty to always start AP.
- Calibrate AZ_STEPS_PER_DEG and EL_STEPS_PER_DEG.
- Verify and adapt pin mapping if your ESPduino-32 variant differs.
- Consider adding debouncing for mechanical endstops or RC filters.
- For safer homing: move slowly, use endstop debounce, or use limit switches with LEDs to test.
*/
