#include <Wire.h>
#include <Arduino.h>
#include "LittleFS.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESPAsyncWebServer.h>

#include "RTClib.h"
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>

// if debug connect to wifi / else create AP
// #define DEBUG

#ifdef DEBUG
#include "wifi.h"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWD;
#else
const char *ssid = "prius";
#endif

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// preferences' ids
#define APP "prius"
#define APP_MAX "max"
#define APP_LIMIT "limit"
#define APP_WAIT "wait"
#define APP_OFFSET "offset"
#define APP_LOW "low"
#define APP_LOWER "lower"
#define APP_MIN "min"

#define BATTERY A0
#define BTN_CHARGE 10
#define RELAY_CHARGE D7
#define BTN_DISCHARGE D8
#define RELAY_DISS1 D6
#define RELAY_DISS2 D3
#define RELAY_DISS3 D5
RTC_DS3231 rtc;
DateTime now;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Preferences preferences;

enum States
{
  IDLE,
  CHARGING,
  CHARGED,
  DISCHARGING,
  DISCHARGED,
  TIMEOUT,
  ERROR
};

States state = IDLE;

// VARS
double vBat = 0.0;
int readingLast = 0;
int readingWait = 50; //  ms
unsigned long readingTimer;

int lowLevel = 3;
double vLow = 150.0;   // level 3
double vLower = 100.0; // level 2
double vMin = 50.0;    // level 1

double vTop = 0.0;
double vOffset = 0.0;
int vTopWait = 600; //  s
unsigned long vTopTimer;

double vMax = 250.0;
bool vMaxHit = false;
int vMaxWait = 300; //  s
unsigned long vMaxTimer;

int timeout = 0;

bool idle = true;
bool charging = false;
bool charged = true;
bool discharging = false;
bool discharged = true;

// button debounce vars
int chargeState = LOW;
int lastChargeState = LOW;
unsigned long chargeDebounce = 0;

int dischargeState = LOW;
int lastDischargeState = LOW;
unsigned long dischargeDebounce = 0;

int debounceDelay = 100;

// used for status update
unsigned long updateTimer;
char dateBuf[20];

bool rtcOnline = false;

// TODO
//  - check log.txt size and delete on startup if bigger than *size*

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    processCommand((char *)data);
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    Serial.println();
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    Serial.println();
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

void setup()
{
  Serial.begin(115200);

  pinMode(BATTERY, INPUT_PULLUP);

  pinMode(BTN_CHARGE, INPUT);
  pinMode(BTN_DISCHARGE, INPUT);

  pinMode(RELAY_CHARGE, OUTPUT);
  digitalWrite(RELAY_CHARGE, LOW);

  pinMode(RELAY_DISS1, OUTPUT);
  digitalWrite(RELAY_DISS1, LOW);
  pinMode(RELAY_DISS2, OUTPUT);
  digitalWrite(RELAY_DISS2, LOW);
  pinMode(RELAY_DISS3, OUTPUT);
  digitalWrite(RELAY_DISS3, LOW);

  Serial.println("Starting ESP");
  Serial.println();

#ifdef DEBUG
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print("Searching for ");
    Serial.println(WIFI_SSID);
  }
  Serial.print("Connected - ");
  Serial.println(WiFi.localIP());
  Serial.println();
#else
  WiFi.softAP(ssid);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
#endif

  if (!rtc.begin())
  {
    Serial.println("RTC missing");
    Serial.println();
  }
  else
  {
    rtc.adjust(DateTime(__DATE__, __TIME__));
    Serial.println("RTC adjust");
    Serial.println();
    rtcOnline = true;
  }

  Serial.println("Starting LittleFS");
  if (!LittleFS.begin())
  {
    Serial.println("An Error has occurred while mounting LittleFS");
    displayError("LittleFS missing");
    return;
  }
  else
  {
    Serial.println("LittleFS loaded");
    deleteFile(LittleFS, "/log.txt");
    logData("SYSTEM STARTING");
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("SSD1306 allocation failed");
    logData("SSD1306 allocation failed");
  }

  Dir dir = LittleFS.openDir("");

  while (dir.next())
  {
    Serial.print(" - ");
    Serial.println(dir.fileName());
    Serial.print("   ");
    Serial.println(dir.fileSize());
  }
  Serial.println("LittleFS done");
  Serial.println();

  Serial.println("Loading defaults");
  preferences.begin(APP, false);
  vMax = preferences.getDouble(APP_MAX, 250.0);
  Serial.print("vMax = ");
  Serial.println(vMax);
  vMaxWait = preferences.getInt(APP_WAIT, 300);
  Serial.print("vMaxWait = ");
  Serial.println(vMaxWait);
  vTopWait = preferences.getInt(APP_LIMIT, 600);
  Serial.print("vTopWait = ");
  Serial.println(vTopWait);
  vOffset = preferences.getDouble(APP_OFFSET, 2);
  Serial.print("vOffset = ");
  Serial.println(vOffset);
  vLow = preferences.getDouble(APP_LOW, 150);
  Serial.print("vLow = ");
  Serial.println(vLow);
  vLower = preferences.getDouble(APP_LOWER, 100);
  Serial.print("vLower = ");
  Serial.println(vLower);
  vMin = preferences.getDouble(APP_MIN, 50);
  Serial.print("vMin = ");
  Serial.println(vMin);
  preferences.end();
  Serial.println("Defaults loaded");
  Serial.println();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html", String(), false); });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/favicon.ico", "image/x-icon"); });

  server.on("/bootstrap.min.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/bootstrap.min.css", "text/css"); });

  server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/styles.css", "text/css"); });

  server.on("/log.txt", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/log.txt", "text/css"); });

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  initWebSocket();

  server.begin();

  Serial.println("ESP initialized");
  Serial.println();
}

void loop()
{
  ws.cleanupClients();

  buttonFunc();
  sensorFunc();
  mainFunc();
  controlFunc();

  updateClient(1);
}

void processCommand(char *json)
{
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  String type = doc["type"];

  // start charging
  if (type == "charge")
  {
    logData("web sent charge");
    if (idle)
    {
      startCharge();
      sendMSG("web started charge - " + String(vBat) + "/" + String(vMax));
    }
    else
      sendMSG("system busy");
  }
  // stop charging
  else if (type == "charged")
  {
    logData("web sent charged");
    if (charging)
    {
      stopCharge();
      sendStatus();
      sendMSG("web stopped charge - " + String(vBat) + "/" + String(vMax));
    }
    else
      sendMSG("system not charging");
  }
  // start discharging
  else if (type == "discharge")
  {
    logData("web sent discharge");
    if (idle)
    {
      startDischarge();
      sendStatus();
      sendMSG("web started discharge - " + String(vBat) + "/" + String(vMin));
    }
    else
      sendMSG("system busy");
  }
  // stop discharging
  else if (type == "discharged")
  {
    logData("web sent discharged");
    if (discharging)
    {
      stopDischarge();
      sendStatus();
      sendMSG("web stopped discharge - " + String(vBat) + "/" + String(vMin));
    }
    else
      sendMSG("system not discharging");
  }
  // fetch config
  else if (type == "fetch")
  {
    sendConfig();
    sendMSG("web fetch config");
  }
  // set config
  else if (type == "config")
  {
    logData("web sent config");
    Serial.println("web sent config");

    JsonObject data = doc["data"];

    Serial.println("Setting preferences");
    preferences.begin(APP, false);

    if (!data["vMax"].isNull())
    {
      String vMaxNew = data["vMax"];
      if (vMax != vMaxNew.toDouble() && vMaxNew.toDouble() > 0)
      {
        vMax = data["vMax"];
        preferences.putDouble(APP_MAX, vMax);
        logData("web vMax = " + String(vMax));
        Serial.println("vMax = " + String(vMax));
      }
    }

    if (!data["vMaxWait"].isNull())
    {
      String vMaxWaitNew = data["vMaxWait"];
      if (vMax != vMaxWaitNew.toInt() && vMaxWaitNew.toInt() > 0)
      {
        vMaxWait = data["vMaxWait"];
        preferences.putInt(APP_WAIT, vMaxWait);
        logData("web vMaxWait = " + String(vMaxWait));
        Serial.println("vMaxWait = " + String(vMaxWait));
      }
    }

    if (!data["vOffset"].isNull())
    {
      String vOffsetNew = data["vOffset"];
      if (vMax != vOffsetNew.toDouble() && vOffsetNew.toDouble() > 0)
      {
        vOffset = data["vOffset"];
        preferences.putDouble(APP_OFFSET, vOffset);
        logData("web vOffset = " + String(vOffset));
        Serial.println("vOffset = " + String(vOffset));
      }
    }

    if (!data["vTopWait"].isNull())
    {
      String vTopWaitNew = data["vTopWait"];
      if (vMax != vTopWaitNew.toInt() && vTopWaitNew.toInt() > 0)
      {
        vTopWait = data["vTopWait"];
        preferences.putInt(APP_LIMIT, vTopWait);
        logData("web vTopWait = " + String(vTopWait));
        Serial.println("vTopWait = " + String(vTopWait));
      }
    }

    if (!data["vLow"].isNull())
    {
      String vLowNew = data["vLow"];
      if (vMax != vLowNew.toDouble() && vLowNew.toDouble() > 0)
      {
        vLow = data["vLow"];
        preferences.putDouble(APP_LOW, vLow);
        logData("web vLow = " + String(vLow));
        Serial.println("vLow = " + String(vLow));
      }
    }

    if (!data["vLower"].isNull())
    {
      String vLowerNew = data["vLower"];
      if (vMax != vLowerNew.toDouble() && vLowerNew.toDouble() > 0)
      {
        vLower = data["vLower"];
        preferences.putDouble(APP_LOWER, vLower);
        logData("web vLower = " + String(vLower));
        Serial.println("vLower = " + String(vLower));
      }
    }

    if (!data["vMin"].isNull())
    {
      String vMinNew = data["vMin"];
      if (vMax != vMinNew.toDouble() && vMinNew.toDouble() > 0)
      {
        vMin = data["vMin"];
        preferences.putDouble(APP_MIN, vMin);
        logData("web vMin = " + String(vMin));
        Serial.println("vMin = " + String(vMin));
      }
    }

    preferences.end();
    Serial.println("Preferences set");

    sendMSG("web config set");
  }
}

void buttonFunc()
{
  int charge = digitalRead(BTN_CHARGE);
  if (charge != lastChargeState)
  {
    chargeDebounce = millis();
  }
  if ((millis() - chargeDebounce) > debounceDelay)
  {
    if (charge != chargeState)
    {
      chargeState = charge;
      if (chargeState == HIGH)
      {
        if (idle)
        {
          startCharge();
          sendMSG("user started charge - " + String(vBat) + "/" + String(vMax));
        }
        else if (charging)
        {
          stopCharge();
          sendMSG("user stopped charge - " + String(vBat) + "/" + String(vMax));
        }
      }
    }
  }
  lastChargeState = charge;

  int discharge = digitalRead(BTN_DISCHARGE);
  if (discharge != lastDischargeState)
  {
    dischargeDebounce = millis();
  }
  if ((millis() - dischargeDebounce) > debounceDelay)
  {
    if (discharge != dischargeState)
    {
      dischargeState = discharge;
      if (dischargeState == HIGH)
      {
        if (idle)
        {
          startDischarge();
          sendMSG("user started discharge - " + String(vBat) + "/" + String(vMin));
        }
        else if (discharging)
        {
          stopDischarge();
          sendMSG("user stopped discharge - " + String(vBat) + "/" + String(vMin));
        }
      }
    }
  }
  lastDischargeState = discharge;
}

void startCharge()
{
  idle = false;
  charging = true;
  charged = false;
  vTop = 0;
  vMaxHit = false;
  state = CHARGING;
}

void stopCharge()
{
  idle = true;
  charging = false;
  charged = false;
  vMaxHit = false;
  state = IDLE;
}

void startDischarge()
{
  idle = false;
  discharged = false;
  discharging = true;
  state = DISCHARGING;
}

void stopDischarge()
{
  idle = true;
  discharged = true;
  discharging = false;
  state = IDLE;
}

void sensorFunc()
{
  unsigned long _time = millis();
  if (_time > readingTimer)
  {
    double voltage = 0;
    int reading = 0;
    for (byte n = 0; n < 5; n++)
    {
      reading += analogRead(BATTERY);
      delay(4);
    }
    // 5 readings - ~2 reading on 0v
    reading = reading / 5 - 5;

    //  average reading * 255 set voltage / 945 reading value
    voltage = (reading * 255 / 945);

    vBat = (vBat + voltage) / 2;

    vBat = constrain(vBat, 0, 300);

    readingTimer = _time + readingWait;
  }
}

void mainFunc()
{
  unsigned long _time = millis();
  if (charging)
  {
    if (vBat > vTop + vOffset)
    {
      vTop = vBat;
      vTopTimer = _time + (vTopWait * 1000);
    }

    if (!vMaxHit)
    {
      if (vBat < vMax)
      {
        if (_time < vTopTimer)
        {
          timeout = (vTopTimer - _time) / 1000;
        }
        else
        {
          charged = true;
          charging = false;
          idle = true;

          state = TIMEOUT;
          sendStatus();
          sendMSG("system can't reach target - " + String(vBat) + "/" + String(vMax));
          sendMSG("system timed out after - " + String(vTopWait) + "s");
        }
      }
      else
      {
        vMaxHit = true;
        vMaxTimer = _time + (vMaxWait * 1000);

        sendStatus();
        sendMSG("system target achieved");
        sendMSG("system waiting " + String(vMaxWait) + "s");
      }
    }
    else
    {
      if (_time < vMaxTimer)
      {
        timeout = (vMaxTimer - _time) / 1000;
      }
      else
      {
        charged = true;
        charging = false;
        vMaxHit = false;
        vTop = 0;
        idle = true;

        state = CHARGED;
        sendStatus();
        sendMSG("system charged - " + String(vBat) + "/" + String(vMax));
      }
    }
  }
  else if (discharging)
  {
    if (vBat > vLow)
    {
      lowLevel = 3;
    }
    else if (vBat > vLower && vBat < vLow)
    {
      lowLevel = 2;
    }
    else if (vBat > vMin && vBat < vLower)
    {
      lowLevel = 1;
    }
    else if (vBat < vMin)
    {
      idle = true;
      discharging = false;
      discharged = true;
      state = DISCHARGED;
      sendStatus();
      sendMSG("system discharged - " + String(vBat) + "/" + String(vMin));
    }
  }
}

void controlFunc()
{
  if (charging)
  {
    digitalWrite(RELAY_CHARGE, HIGH);
  }
  else
  {
    digitalWrite(RELAY_CHARGE, LOW);
  }
  if (discharging)
  {
    if (lowLevel == 3)
    {
      digitalWrite(RELAY_DISS3, HIGH);
      digitalWrite(RELAY_DISS2, HIGH);
      digitalWrite(RELAY_DISS1, HIGH);
    }
    else if (lowLevel == 2)
    {
      digitalWrite(RELAY_DISS3, LOW);
      digitalWrite(RELAY_DISS2, HIGH);
      digitalWrite(RELAY_DISS1, HIGH);
    }
    else
    {
      digitalWrite(RELAY_DISS3, LOW);
      digitalWrite(RELAY_DISS2, LOW);
      digitalWrite(RELAY_DISS1, HIGH);
    }
  }
  else
  {
    digitalWrite(RELAY_DISS3, LOW);
    digitalWrite(RELAY_DISS2, LOW);
    digitalWrite(RELAY_DISS1, LOW);
  }
}

String getTimeString()
{
  if (rtcOnline)
  {
    now = rtc.now();

    sprintf(dateBuf, "%02d/%02d/%02d %02d:%02d:%02d",
            now.year(),
            now.month(),
            now.day(),
            now.hour(),
            now.minute(),
            now.second());

    return String(dateBuf) + " - ";
  }
  return String(millis() / 1000) + " - ";
}

void updateClient(int delay)
{
  unsigned long _time = millis();
  if (_time > updateTimer)
  {
    sendStatus();
    updateDisplay();
    updateTimer = _time + (delay * 1000);
  }
}

void sendMSG(String msg)
{
  logData(msg);
  String send = "";
  StaticJsonDocument<400> json;
  json["type"] = "log";
  json["data"] = getTimeString() + msg;

  serializeJson(json, send);
  ws.textAll(send);
}

void sendStatus()
{
  String send = "";
  StaticJsonDocument<400> json;
  json["type"] = "status";

  int rnd = random(50, 250);
  int rnd2 = random(0, 100);

  JsonObject jsonData = json.createNestedObject("data");

  jsonData["state"] = state;

  jsonData["vBat"] = String(vBat, 2);

  jsonData["vLow"] = String(vLow, 2);
  jsonData["vLower"] = String(vLower, 2);
  jsonData["vMin"] = String(vMin, 2);

  jsonData["vTop"] = String(vTop, 2);
  jsonData["vOffset"] = String(vOffset, 2);
  jsonData["vTopWait"] = vTopWait;

  jsonData["vMax"] = String(vMax, 2);
  jsonData["vMaxHit"] = String(vMaxHit, 2);
  jsonData["vMaxWait"] = vMaxWait;

  jsonData["timeout"] = true;
  jsonData["timer"] = timeout;

  serializeJson(json, send);
  ws.textAll(send);
}

void updateDisplay()
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);

  String stateS = state == IDLE          ? "idle"
                  : state == CHARGING    ? "charging"
                  : state == CHARGED     ? "charged "
                  : state == DISCHARGING ? "discharging"
                  : state == DISCHARGED  ? "discharged "
                  : state == TIMEOUT     ? "timeout "
                                         : "error";

  drawCenterString(stateS.c_str(), 64, 0);

  String bat = String(vBat);
  String max = String(vBat) + "/" + String(vMax);
  String min = String(vBat) + "/" + String(vMin);
  String tim = String(timeout);

  if (idle)
    drawCenterString(bat.c_str(), 64, 10);
  else if (charging || charged)
  {
    drawCenterString(max.c_str(), 64, 10);
    drawCenterString(tim.c_str(), 64, 20);
  }
  else if (discharging || discharged)
    drawCenterString(min.c_str(), 64, 10);

  display.display();
}

void displayError(String error)
{
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.print(error);
  display.display();
}

void drawCenterString(String text, int x, int y)
{
  display.setCursor(x - text.length() * 3, y);
  display.print(text);
}

void sendConfig()
{
  String send = "";
  StaticJsonDocument<400> json;
  json["type"] = "config";

  JsonObject jsonData = json.createNestedObject("data");
  jsonData["vLow"] = String(vLow, 2);
  jsonData["vLower"] = String(vLower, 2);
  jsonData["vMin"] = String(vMin, 2);

  jsonData["vOffset"] = String(vOffset, 2);
  jsonData["vTopWait"] = vTopWait;

  jsonData["vMax"] = String(vMax, 2);
  jsonData["vMaxWait"] = vMaxWait;

  serializeJson(json, send);
  ws.textAll(send);
}

void logData(String data)
{
  String timedData = getTimeString() + data;
  const char *dt = timedData.c_str();
  appendFile(LittleFS, "/log.txt", dt);
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  File file = fs.open(path, "a");
  if (!file)
  {
    Serial.println(String(message) + " - failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    file.print("\n");
  }
  else
  {
    Serial.println(String(message) + " - append failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char *path)
{
  Serial.printf("Deleting file: %s\r\n", path);
  if (fs.remove(path))
  {
    Serial.println(String(path) + " - file deleted");
  }
  else
  {
    Serial.println(String(path) + " - delete failed");
  }
}
