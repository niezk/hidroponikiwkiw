// Blynk Template - Hardcoded
#define BLYNK_TEMPLATE_ID "TMPL6KuOhWVBP"
#define BLYNK_TEMPLATE_NAME "HIDROPONIK V1"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

// Sensor pins
#define TDS_SENSOR 33
#define TEMP_SENSOR 32 // DS18B20 data pin
#define PH_SENSOR 35

// Relay pins - FIXED ASSIGNMENTS
#define IN1 5  // pH+ (increase pH)
#define IN2 18 // pH- (decrease pH)
#define IN4 23 // Vitamin (TDS control) - IN3 removed

// LCD I2C address (usually 0x27 or 0x3F)
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Blynk Virtual Pins
#define VPIN_TDS V0
#define VPIN_TEMP V1
#define VPIN_PH V2
#define VPIN_RELAY1 V3 // pH+
#define VPIN_RELAY2 V4 // pH-
#define VPIN_RELAY4 V6 // Vitamin (V5 removed for IN3)

// Threshold Virtual Pins
#define VPIN_TDS_THRESHOLD V7
#define VPIN_TDS_MAX_THRESHOLD V8
#define VPIN_PH_MIN_THRESHOLD V9
#define VPIN_PH_MAX_THRESHOLD V10

// Automation Enable Virtual Pins
#define VPIN_AUTO_TDS V11
#define VPIN_AUTO_TEMP V12
#define VPIN_AUTO_PH V13

// Status LED Virtual Pin
#define VPIN_STATUS_LED V17

// LED indicators for automation active status
#define VPIN_LED_AUTO_TDS V14 // LED for TDS automation active
#define VPIN_LED_AUTO_PH V15  // LED for pH automation active

// Setup OneWire and DallasTemperature
OneWire oneWire(TEMP_SENSOR);
DallasTemperature tempSensor(&oneWire);

// Setup LCD
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Forward declarations
void startConfigPortal();
void connectToWiFi();
void handleConfigPage();
void handleSave();
void handleStatusPage();
void handleLogout();
void readSensors();
void checkAutomation();
void updateLCD();
void handleNotFound();
void updateAutoLEDs();

// Web server
WebServer server(80);
Preferences preferences;

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

// WiFi and Blynk credentials
String ssid = "";
String password = "";
String blynkToken = "";

bool wifiConfigMode = false;

// Sensor values
float tdsValue = 0;
float tempValue = 0;
float phValue = 0;

// Relay states
bool relay1State = false; // pH+
bool relay2State = false; // pH-
bool relay4State = false; // Vitamin

// Threshold values (defaults)
float tdsMaxThresold = 700.0;
float tdsThreshold = 500.0; // ppm
float tempThreshold = 30.0; // °C
float phMinThreshold = 6.5; // pH
float phMaxThreshold = 8.5; // pH

// Automation enable flags
bool autoTDS = false;
bool autoTemp = false;
bool autoPH = false;

// Manual override flags (prevent automation if manually controlled)
bool relay1Manual = false;
bool relay2Manual = false;
bool relay4Manual = false;

// Logo
byte name0x1[] = {B00000, B00001, B00000, B00110, B01110, B11110, B11110, B00001};
byte name0x0[] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B00010};
byte name0x2[] = {B00000, B10000, B00000, B01100, B01110, B01111, B01111, B10000};
byte name0x3[] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B01000};
byte name1x0[] = {B00010, B00000, B00000, B00000, B00000, B00000, B00000, B00000};
byte name1x1[] = {B00001, B11110, B11110, B01110, B00110, B00000, B00001, B00000};
byte name1x2[] = {B10000, B01111, B01111, B01110, B01100, B00000, B10000, B00000};
byte name1x3[] = {B01000, B00000, B00000, B00000, B00000, B00000, B00000, B00000};

// verify
byte checkedlist[] = {B00000, B00001, B00011, B10110, B11100, B01000, B00000, B00000};
// plant
byte plant[] = {B00000, B01010, B01110, B01110, B00100, B10100, B10101, B01110};

// Timer for sensor reading and LCD update
BlynkTimer timer;

// LCD display mode (0: pH/Temp, 1: TDS)
int lcdDisplayMode = 0;
unsigned long lastLcdSwitch = 0;
#define LCD_SWITCH_INTERVAL 3000 // Switch display every 3 seconds

void setup()
{
  Serial.begin(115200);

  Serial.println("Starting...");

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Create custom characters for logo
  lcd.createChar(0, name0x0);
  lcd.createChar(1, name0x1);
  lcd.createChar(2, name0x2);
  lcd.createChar(3, name0x3);
  lcd.createChar(4, name1x0);
  lcd.createChar(5, name1x1);
  lcd.createChar(6, name1x2);
  lcd.createChar(7, name1x3);

  // Display logo
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write(0);
  lcd.write(1);
  lcd.write(2);
  lcd.write(3);
  lcd.print(" Hidroponik");
  lcd.setCursor(0, 1);
  lcd.write(4);
  lcd.write(5);
  lcd.write(6);
  lcd.write(7);
  lcd.print("  Starting...");
  delay(2000);

  // Initialize relay pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set all relays to OFF initially (HIGH = OFF for most relay modules)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, HIGH);

  // Initialize DS18B20 temperature sensor
  tempSensor.begin();
  Serial.print("DS18B20 devices found: ");
  Serial.println(tempSensor.getDeviceCount());

  // Initialize analog sensor pins as INPUT
  pinMode(TDS_SENSOR, INPUT);
  pinMode(PH_SENSOR, INPUT);

  // Load saved credentials
  preferences.begin("wifi-config", false);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  blynkToken = preferences.getString("blynk", "");

  // Load automation settings
  tdsThreshold = preferences.getFloat("tdsThresh", 500.0);
  tdsMaxThresold = preferences.getFloat("tempThresh", 30.0);
  phMinThreshold = preferences.getFloat("phMinThresh", 6.5);
  phMaxThreshold = preferences.getFloat("phMaxThresh", 8.5);
  autoTDS = preferences.getBool("autoTDS", false);
  autoTemp = preferences.getBool("autoTemp", false);
  autoPH = preferences.getBool("autoPH", false);

  preferences.end();

  // Check if credentials are saved
  if (ssid == "" || blynkToken == "")
  {
    startConfigPortal();
  }
  else
  {
    connectToWiFi();
  }
}

void loop()
{
  if (wifiConfigMode)
  {
    dnsServer.processNextRequest();
    server.handleClient();
  }
  else
  {
    server.handleClient();
    Blynk.run();
    timer.run();

    // Update LCD display periodically
    if (millis() - lastLcdSwitch >= LCD_SWITCH_INTERVAL)
    {
      updateLCD();
      lastLcdSwitch = millis();
    }
  }
}

void startConfigPortal()
{
  wifiConfigMode = true;

  WiFi.mode(WIFI_AP);
  WiFi.softAP("Hidroponik-Setup", "tme12345");
  IPAddress apIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gateway, subnet);

  Serial.println("Config Mode");
  Serial.println("WiFi: Hidroponik-Setup");
  Serial.println("Pass: tme12345");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  lcd.clear();
  lcd.print(" System OK!");
  lcd.setCursor(0, 1);
  lcd.print("IP:");
  lcd.print(WiFi.softAPIP());

  dnsServer.start(DNS_PORT, "*", apIP);
  Serial.println("DNS Server started for captive portal");

  server.on("/", handleConfigPage);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/config", handleConfigPage);
  server.on("/generate_204", handleConfigPage);
  server.on("/fwlink", handleConfigPage);
  server.onNotFound(handleNotFound);
  server.begin();
}

void handleNotFound()
{
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleConfigPage()
{
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial; max-width: 400px; margin: 50px auto; padding: 20px; }";
  html += "h1 { text-align: center; color: #333; }";
  html += "label { display: block; margin: 15px 0 5px; font-weight: bold; }";
  html += "input { width: 100%; padding: 10px; font-size: 16px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }";
  html += "button { background-color: #4CAF50; color: white; padding: 14px; margin: 20px 0; border: none; ";
  html += "cursor: pointer; width: 100%; font-size: 16px; border-radius: 4px; }";
  html += "button:hover { background-color: #45a049; }";
  html += ".info { background-color: #e7f3fe; padding: 15px; border-left: 4px solid #2196F3; margin: 20px 0; }";
  html += "</style></head><body>";
  html += "<h1>ESP32 WiFi Setup</h1>";
  html += "<div class='info'>Configure WiFi and Blynk credentials<br><small>Template: " + String(BLYNK_TEMPLATE_NAME) + "</small></div>";
  html += "<form action='/save' method='POST'>";
  html += "<label>WiFi SSID:</label>";
  html += "<input type='text' name='ssid' required placeholder='Your WiFi Name'>";
  html += "<label>WiFi Password:</label>";
  html += "<input type='password' name='password' placeholder='Your WiFi Password'>";
  html += "<label>Blynk Auth Token:</label>";
  html += "<input type='text' name='blynk' value='n-A-rFH2W9mSEBg2yzOXqCEz8qiGysvJ' required placeholder='Your Auth Token'>";
  html += "<button type='submit'>Save & Connect</button>";
  html += "</form>";
  html += "<div class='info' style='background-color: #fff3cd; border-left-color: #ffc107;'>";
  html += "<small><b>Note:</b> Get Auth Token from Blynk.Console → Device Info</small></div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleSave()
{
  if (server.hasArg("ssid") && server.hasArg("password") && server.hasArg("blynk"))
  {
    ssid = server.arg("ssid");
    password = server.arg("password");
    blynkToken = server.arg("blynk");

    preferences.begin("wifi-config", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("blynk", blynkToken);
    preferences.end();

    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body { font-family: Arial; text-align: center; margin: 50px; }</style>";
    html += "</head><body><h1>Saved!</h1><p>ESP32 will now connect to WiFi...</p></body></html>";
    server.send(200, "text/html", html);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Config Saved!");
    lcd.setCursor(0, 1);
    lcd.print("Restarting...");

    delay(2000);
    ESP.restart();
  }
  else
  {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void handleStatusPage()
{
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<style>";
  html += "body { font-family: Arial; max-width: 500px; margin: 50px auto; padding: 20px; }";
  html += "h1 { text-align: center; color: #333; }";
  html += "h2 { color: #555; border-bottom: 2px solid #4CAF50; padding-bottom: 10px; }";
  html += ".status { background-color: #d4edda; padding: 15px; border-left: 4px solid #28a745; margin: 20px 0; border-radius: 4px; }";
  html += ".info { background-color: #f8f9fa; padding: 10px; margin: 10px 0; border-radius: 4px; }";
  html += ".sensor { background-color: #e3f2fd; padding: 15px; margin: 10px 0; border-radius: 4px; border-left: 4px solid #2196F3; }";
  html += ".relay { background-color: #fff3e0; padding: 10px; margin: 5px 0; border-radius: 4px; display: flex; justify-content: space-between; align-items: center; }";
  html += ".relay-on { background-color: #c8e6c9; border-left: 4px solid #4CAF50; }";
  html += ".relay-off { background-color: #ffcdd2; border-left: 4px solid #f44336; }";
  html += ".auto-status { background-color: #fff9e6; padding: 10px; margin: 10px 0; border-radius: 4px; border-left: 4px solid #ffc107; }";
  html += ".logout-btn { background-color: #dc3545; color: white; padding: 14px; margin: 20px 0; border: none; ";
  html += "cursor: pointer; width: 100%; font-size: 16px; border-radius: 4px; text-decoration: none; display: block; text-align: center; }";
  html += ".logout-btn:hover { background-color: #c82333; }";
  html += ".warning { background-color: #fff3cd; padding: 15px; border-left: 4px solid #ffc107; margin: 20px 0; }";
  html += ".relay-label { font-size: 11px; color: #666; }";
  html += "</style></head><body>";
  html += "<h1>ESP32 Status</h1>";
  html += "<div class='status'><strong>✓ Connected</strong></div>";

  html += "<h2>Connection Info</h2>";
  html += "<div class='info'><strong>WiFi SSID:</strong> " + ssid + "</div>";
  html += "<div class='info'><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</div>";
  html += "<div class='info'><strong>Template:</strong> " + String(BLYNK_TEMPLATE_NAME) + "</div>";
  html += "<div class='info'><strong>Template ID:</strong> " + String(BLYNK_TEMPLATE_ID) + "</div>";
  html += "<div class='info'><strong>Blynk Status:</strong> " + String(Blynk.connected() ? "Connected" : "Disconnected") + "</div>";

  html += "<h2>Sensor Readings</h2>";
  html += "<div class='sensor'><strong>TDS:</strong> " + String(tdsValue, 2) + " ppm</div>";
  html += "<div class='sensor'><strong>Temperature (DS18B20):</strong> " + String(tempValue, 2) + " °C</div>";
  html += "<div class='sensor'><strong>pH:</strong> " + String(phValue, 2) + "</div>";

  html += "<h2>Automation Status</h2>";
  html += "<div class='auto-status'><strong>TDS Auto:</strong> " + String(autoTDS ? "ENABLED" : "DISABLED") + " (Threshold: " + String(tdsThreshold, 1) + " ppm)<br><span class='relay-label'>Controls: Relay 4 (Vitamin)</span></div>";
  html += "<div class='auto-status'><strong>Temp Auto:</strong> " + String(autoTemp ? "ENABLED" : "DISABLED") + " (Threshold: " + String(tempThreshold, 1) + " °C)</div>";
  html += "<div class='auto-status'><strong>pH Auto:</strong> " + String(autoPH ? "ENABLED" : "DISABLED") + " (Range: " + String(phMinThreshold, 1) + "-" + String(phMaxThreshold, 1) + ")<br><span class='relay-label'>Controls: Relay 1 (pH+) & Relay 2 (pH-)</span></div>";

  html += "<h2>Relay Status</h2>";
  html += "<div class='relay " + String(relay1State ? "relay-on" : "relay-off") + "'><div><strong>Relay 1 (pH+):</strong><br><span class='relay-label'>Increase pH</span></div><span>" + String(relay1State ? "ON" : "OFF") + "</span></div>";
  html += "<div class='relay " + String(relay2State ? "relay-on" : "relay-off") + "'><div><strong>Relay 2 (pH-):</strong><br><span class='relay-label'>Decrease pH</span></div><span>" + String(relay2State ? "ON" : "OFF") + "</span></div>";
  html += "<div class='relay " + String(relay4State ? "relay-on" : "relay-off") + "'><div><strong>Relay 4 (Vitamin):</strong><br><span class='relay-label'>TDS Control</span></div><span>" + String(relay4State ? "ON" : "OFF") + "</span></div>";

  html += "<div class='warning'><strong>⚠ Warning:</strong> Logging out will erase all credentials and restart the device in configuration mode.</div>";
  html += "<a href='/logout' class='logout-btn'>Logout & Reset</a>";
  html += "<p style='text-align: center; color: #999; font-size: 12px;'>Page auto-refreshes every 5 seconds</p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleLogout()
{
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body { font-family: Arial; text-align: center; margin: 50px; }</style>";
  html += "</head><body><h1>Logging Out...</h1><p>All credentials cleared. Device restarting in config mode...</p></body></html>";
  server.send(200, "text/html", html);

  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Logging Out...");
  lcd.setCursor(0, 1);
  lcd.print("Resetting...");

  preferences.begin("wifi-config", false);
  preferences.clear();
  preferences.end();

  Serial.println("Credentials cleared. Restarting...");

  delay(1000);
  ESP.restart();
}

void connectToWiFi()
{
  Serial.println("Connecting to WiFi...");
  Serial.println(ssid);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1);
  lcd.print(ssid);

  WiFi.mode(WIFI_STA);
  if (password == "" || password.length() == 0)
  {
    Serial.println(password);
    Serial.println(ssid);
    WiFi.begin(ssid.c_str());
  }
  else
  {
    WiFi.begin(ssid.c_str(), password.c_str());
  }

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    lcd.setCursor(15, 1);
    lcd.print(attempts % 2 == 0 ? "." : " ");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected!");
    lcd.setCursor(0, 1);
    lcd.print("IP:");
    lcd.print(WiFi.localIP());
    delay(3000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connecting");
    lcd.setCursor(0, 1);
    lcd.print("Blynk...");

    Blynk.config(blynkToken.c_str());
    Blynk.connect();

    Serial.println("Blynk Connected!");
    Serial.println("Device Ready!");
    Serial.println("Relay Configuration:");
    Serial.println("- Relay 1 (IN1): pH+ (Increase pH)");
    Serial.println("- Relay 2 (IN2): pH- (Decrease pH)");
    Serial.println("- Relay 4 (IN4): Vitamin (TDS control)");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Ready!");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");
    delay(2000);

    timer.setInterval(2000L, readSensors);

    server.on("/", handleStatusPage);
    server.on("/logout", handleLogout);
    server.begin();

    Serial.print("Web server started at http://");
    Serial.println(WiFi.localIP());

    updateLCD();
  }
  else
  {
    Serial.println("\nWiFi Connection Failed!");
    Serial.println("Restarting config...");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed!");
    lcd.setCursor(0, 1);
    lcd.print("Restarting...");

    delay(3000);

    preferences.begin("wifi-config", false);
    preferences.clear();
    preferences.end();
    ESP.restart();
  }
}

void readSensors()
{
  // Read TDS sensor
  int tdsRaw = analogRead(TDS_SENSOR);
  tdsValue = (tdsRaw * 3.3 / 4095.0) * 1000;

  // Read DS18B20 Temperature sensor
  tempSensor.requestTemperatures();
  tempValue = tempSensor.getTempCByIndex(0);

  if (tempValue == DEVICE_DISCONNECTED_C || tempValue == 85.0)
  {
    Serial.println("Error: Could not read temperature from DS18B20");
    tempValue = 0.0;
  }

  // Read pH sensor
  int phRaw = analogRead(PH_SENSOR);
  phValue = (phRaw * 3.3 / 4095.0) * 3.5 + 0;

  // Send to Blynk
  Blynk.virtualWrite(VPIN_TDS, tdsValue);
  Blynk.virtualWrite(VPIN_TEMP, tempValue);
  Blynk.virtualWrite(VPIN_PH, phValue);

  // Check automation rules
  checkAutomation();

  // Update automation LEDs
  updateAutoLEDs();

  // Print to Serial
  Serial.println("--- Sensor Readings ---");
  Serial.print("TDS: ");
  Serial.print(tdsValue);
  Serial.println(" ppm");
  Serial.print("Temp (DS18B20): ");
  Serial.print(tempValue);
  Serial.println(" °C");
  Serial.print("pH: ");
  Serial.println(phValue);
}

void updateLCD()
{
  lcd.clear();
  lcd.createChar(9, plant);
  byte name0x1[] = {B01100, B01110, B00111, B00111, B00011, B00001, B00010, B00100};
  byte name0x2[] = {B00110, B01110, B11100, B11100, B11000, B10000, B01000, B00100};
  byte name0x13[] = {B01100, B01110, B00111, B00111, B00011, B00001, B00010, B00100};
  byte name0x14[] = {B00110, B01110, B11100, B11100, B11000, B10000, B01000, B00100};

  if (lcdDisplayMode == 0)
  {
    lcd.setCursor(1, 0);
    lcd.write(9);
    lcd.setCursor(2, 0);
    lcd.print(">pH:");
    lcd.print(phValue, 2);
    lcd.print(" ");

    lcd.setCursor(1, 1);
    lcd.write(9);
    lcd.setCursor(2, 1);
    lcd.print(">Temp:");
    lcd.print(tempValue, 1);
    lcd.print("C");

    lcdDisplayMode = 1;
  }
  else
  {
    lcd.createChar(1, name0x1);
    lcd.setCursor(0, 0);
    lcd.write(1);

    lcd.createChar(2, name0x2);
    lcd.setCursor(1, 0);
    lcd.write(2);

    lcd.createChar(3, name0x13);
    lcd.setCursor(14, 0);
    lcd.write(3);

    lcd.createChar(4, name0x14);
    lcd.setCursor(15, 0);
    lcd.write(4);
    lcd.setCursor(2, 0);
    lcd.print(" Hidroponik ");
    lcd.createChar(5, plant);

    lcd.setCursor(1, 1);
    lcd.write(5);
    lcd.setCursor(2, 1);
    lcd.print(">TDS:");
    lcd.print(tdsValue, 0);
    lcd.print(" ppm");

    lcdDisplayMode = 0;
  }
}

// Update automation status LEDs
void updateAutoLEDs()
{
  // LED for TDS automation - lights up when automation is enabled AND relay is active
  if (autoTDS && relay4State)
  {
    Blynk.virtualWrite(VPIN_LED_AUTO_TDS, 255); // LED ON (bright)
  }
  else
  {
    Blynk.virtualWrite(VPIN_LED_AUTO_TDS, 0); // LED OFF
  }

  // LED for pH automation - lights up when automation is enabled AND any pH relay is active
  if (autoPH && (relay1State || relay2State))
  {
    Blynk.virtualWrite(VPIN_LED_AUTO_PH, 255); // LED ON (bright)
  }
  else
  {
    Blynk.virtualWrite(VPIN_LED_AUTO_PH, 0); // LED OFF
  }
}

void checkAutomation()
{
  // pH Automation - Controls Relay 1 (pH+) and Relay 2 (pH-)
  if (autoPH)
  {
    // If pH is too LOW, activate pH+ (Relay 1)
    if (phValue < phMinThreshold)
    {
      if (!relay1Manual && !relay1State)
      {
        relay1State = true;
        digitalWrite(IN1, LOW);
        Blynk.virtualWrite(VPIN_RELAY1, relay1State);
        Serial.println("AUTO: pH too LOW - pH+ (Relay 1) ON");
      }
      // Turn OFF pH- if it's on
      if (!relay2Manual && relay2State)
      {
        relay2State = false;
        digitalWrite(IN2, HIGH);
        Blynk.virtualWrite(VPIN_RELAY2, relay2State);
        Serial.println("AUTO: pH- (Relay 2) OFF");
      }
    }
    // If pH is too HIGH, activate pH- (Relay 2)
    else if (phValue > phMaxThreshold)
    {
      if (!relay2Manual && !relay2State)
      {
        relay2State = true;
        digitalWrite(IN2, LOW);
        Blynk.virtualWrite(VPIN_RELAY2, relay2State);
        Serial.println("AUTO: pH too HIGH - pH- (Relay 2) ON");
      }
      // Turn OFF pH+ if it's on
      if (!relay1Manual && relay1State)
      {
        relay1State = false;
        digitalWrite(IN1, HIGH);
        Blynk.virtualWrite(VPIN_RELAY1, relay1State);
        Serial.println("AUTO: pH+ (Relay 1) OFF");
      }
    }
    // pH is within range, turn OFF both
    else
    {
      if (!relay1Manual && relay1State)
      {
        relay1State = false;
        digitalWrite(IN1, HIGH);
        Blynk.virtualWrite(VPIN_RELAY1, relay1State);
        Serial.println("AUTO: pH OK - pH+ (Relay 1) OFF");
      }
      if (!relay2Manual && relay2State)
      {
        relay2State = false;
        digitalWrite(IN2, HIGH);
        Blynk.virtualWrite(VPIN_RELAY2, relay2State);
        Serial.println("AUTO: pH OK - pH- (Relay 2) OFF");
      }
    }
  }

  // TDS Automation - Controls Relay 4 (Vitamin) only
  if (autoTDS)
  {
    bool shouldActivate = ((tdsValue + 75) < (tdsThreshold - 5)); // Activate if TDS is LOW (needs vitamins)

    // Control Relay 4 (Vitamin)
    if (!relay4Manual)
    {
      if (relay4State != shouldActivate)
      {
        relay4State = shouldActivate;
        digitalWrite(IN4, !relay4State); // Invert for relay module
        Blynk.virtualWrite(VPIN_RELAY4, relay4State);
        Serial.println("AUTO: TDS - Vitamin (Relay 4) " + String(relay4State ? "ON" : "OFF"));
      }
    }
  }
}

// Blynk Virtual Pin handlers for Relays
BLYNK_WRITE(VPIN_RELAY1)
{
  relay1State = param.asInt();
  digitalWrite(IN1, !relay1State);
  relay1Manual = true;
  Serial.print("MANUAL: Relay 1 (pH+): ");
  Serial.println(relay1State ? "ON" : "OFF");
  updateAutoLEDs();
}

BLYNK_WRITE(VPIN_RELAY2)
{
  relay2State = param.asInt();
  digitalWrite(IN2, !relay2State);
  relay2Manual = true;
  Serial.print("MANUAL: Relay 2 (pH-): ");
  Serial.println(relay2State ? "ON" : "OFF");
  updateAutoLEDs();
}

BLYNK_WRITE(VPIN_RELAY4)
{
  relay4State = param.asInt();
  digitalWrite(IN4, !relay4State);
  relay4Manual = true;
  Serial.print("MANUAL: Relay 4 (Vitamin): ");
  Serial.println(relay4State ? "ON" : "OFF");
  updateAutoLEDs();
}

// Threshold handlers
BLYNK_WRITE(VPIN_TDS_THRESHOLD)
{
  tdsThreshold = param.asFloat();
  preferences.begin("wifi-config", false);
  preferences.putFloat("tdsThresh", tdsThreshold);
  preferences.end();
  Serial.print("TDS Threshold set to: ");
  Serial.println(tdsThreshold);
}

BLYNK_WRITE(VPIN_TEMP_THRESHOLD)
{
  tempThreshold = param.asFloat();
  preferences.begin("wifi-config", false);
  preferences.putFloat("tempThresh", tempThreshold);
  preferences.end();
  Serial.print("Temp Threshold set to: ");
  Serial.println(tempThreshold);
}

BLYNK_WRITE(VPIN_PH_MIN_THRESHOLD)
{
  phMinThreshold = param.asFloat();
  preferences.begin("wifi-config", false);
  preferences.putFloat("phMinThresh", phMinThreshold);
  preferences.end();
  Serial.print("pH Min Threshold set to: ");
  Serial.println(phMinThreshold);
}

BLYNK_WRITE(VPIN_PH_MAX_THRESHOLD)
{
  phMaxThreshold = param.asFloat();
  preferences.begin("wifi-config", false);
  preferences.putFloat("phMaxThresh", phMaxThreshold);
  preferences.end();
  Serial.print("pH Max Threshold set to: ");
  Serial.println(phMaxThreshold);
}

// Automation enable handlers
BLYNK_WRITE(VPIN_AUTO_TDS)
{
  autoTDS = param.asInt();
  preferences.begin("wifi-config", false);
  preferences.putBool("autoTDS", autoTDS);
  preferences.end();
  if (autoTDS)
  {
    relay4Manual = false; // Clear manual override when automation is enabled
  }
  Serial.print("TDS Automation: ");
  Serial.println(autoTDS ? "ENABLED" : "DISABLED");
  updateAutoLEDs();
}

BLYNK_WRITE(VPIN_AUTO_TEMP)
{
  autoTemp = param.asInt();
  preferences.begin("wifi-config", false);
  preferences.putBool("autoTemp", autoTemp);
  preferences.end();
  Serial.print("Temp Automation: ");
  Serial.println(autoTemp ? "ENABLED" : "DISABLED");
}

BLYNK_WRITE(VPIN_AUTO_PH)
{
  autoPH = param.asInt();
  preferences.begin("wifi-config", false);
  preferences.putBool("autoPH", autoPH);
  preferences.end();
  if (autoPH)
  {
    relay1Manual = false; // Clear manual override when automation is enabled
    relay2Manual = false;
  }
  Serial.print("pH Automation: ");
  Serial.println(autoPH ? "ENABLED" : "DISABLED");
  updateAutoLEDs();
}

// Sync relay states when Blynk connects
BLYNK_CONNECTED()
{
  // Request the latest state from Blynk server
  Blynk.syncVirtual(VPIN_RELAY1);
  Blynk.syncVirtual(VPIN_RELAY2);
  Blynk.syncVirtual(VPIN_RELAY4);

  // Send current threshold values to Blynk
  Blynk.virtualWrite(VPIN_TDS_THRESHOLD, tdsThreshold);
  Blynk.virtualWrite(VPIN_TDS_MAX_THRESHOLD, tempThreshold);
  Blynk.virtualWrite(VPIN_PH_MIN_THRESHOLD, phMinThreshold);
  Blynk.virtualWrite(VPIN_PH_MAX_THRESHOLD, phMaxThreshold);

  // Send automation states
  Blynk.virtualWrite(VPIN_AUTO_TDS, autoTDS);
  Blynk.virtualWrite(VPIN_AUTO_TEMP, autoTemp);
  Blynk.virtualWrite(VPIN_AUTO_PH, autoPH);

  // Turn on status LED
  Blynk.virtualWrite(VPIN_STATUS_LED, 255);

  // Update automation LEDs
  updateAutoLEDs();

  Serial.println("Blynk connected and synced!");
}
