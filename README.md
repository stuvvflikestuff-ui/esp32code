# esp32code
below contains the whole code for the esp32 that controls all of the sensors and components connected and also has the IoT connected to the companion application (MUST BE IN CODE VIEW)

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// === Pin Definitions ===
// Sensors
const int turbidity_Pin = 34;  // Turbidity sensor on GPIO34 (ADC1)
const int oneWireBus = 15;     // DS18B20 data pin

// Added pins for pH and ammonia sensors
const int pH_sensor_Pin = 32;      // Analog pin for pH4502C sensor
const int ammonia_sensor_Pin = 33; // Analog pin for MQ-137 sensor

// Servo and Motor pins
const int servoPin = 17; // Servo PWM pin
const int MOTOR_PWM_R = 25;   // PWM pin for right motor
const int MOTOR_PWM_L = 26;   // PWM pin for left motor
const int MOTOR_EN_R = 27;    // Enable pin for right motor
const int MOTOR_EN_L = 14;    // Enable pin for left motor

// PWM channels for ESP32
const int PWM_CHANNEL_R = 0;
const int PWM_CHANNEL_L = 1;
const int PWM_FREQ = 50000; // 50 kHz PWM frequency
const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)

// === LCD Setup ===
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C LCD at address 0x27

// === DS18B20 Setup ===
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// === WiFi Credentials ===
const char* ssid = "Converge_2.4GHz_dTeuFh";
const char* password = "HN37ANRQ";

// === Server URLs ===
const char* serverName = "http://192.168.1.17:3000/api/data";
const char* feedCommandUrl = "http://192.168.1.17:3000/api/feedingCommand";

// === Variables ===
float pH_value;
float ammoniaLevel;
float turbidity_voltage;
float temperature;

// Servo object
Servo myservo;

// Motor control state
unsigned long spreadStartTime = 0;
unsigned long spreadDuration = 20000; // 20 seconds spreading duration
bool isSpreading = false;

// Feeding command state to avoid repeated execution
bool feedingInProgress = false;

// Calibration constants for MQ-137 (adjust after calibration)
const float RL_VALUE = 10.0; // Load resistance in kilo ohms
float R0 = 10.0;             // Sensor resistance in clean air (calibrate this)

// Utility function to generate random float
float randomFloat(float minVal, float maxVal) {
  return minVal + ((float)random(0, 10000) / 10000.0) * (maxVal - minVal);
}

// Function to read pH from pH4502C sensor
float readPH() {
  int rawValue = analogRead(pH_sensor_Pin);
  float voltage = rawValue * (3.3 / 4095.0);

  // Calibration parameters (adjust after calibration)
  float Vneutral = 2.5;  // Voltage at pH 7
  float slope = 0.18;    // Voltage change per pH unit

  float pH = 7 - ((voltage - Vneutral) / slope);
  return pH;
}

// Function to read ammonia concentration from MQ-137 sensor
float readAmmonia() {
  int rawValue = analogRead(ammonia_sensor_Pin);
  float voltage = rawValue * (3.3 / 4095.0);

  // Calculate sensor resistance Rs
  float Rs = ((3.3 - voltage) / voltage) * RL_VALUE;

  // Calculate ratio Rs/R0
  float ratio = Rs / R0;

  // Convert ratio to ppm using example formula (calibrate for your sensor)
  float ppm = pow(10, ((log10(ratio) - 0.5) / -0.45));

  return ppm;
}

// Motor control functions
void controlledSpread() {
  Serial.println("Starting controlled spread (600 RPM)");
  digitalWrite(MOTOR_EN_R, HIGH);
  digitalWrite(MOTOR_EN_L, HIGH);
  ledcWrite(PWM_CHANNEL_R, 127);  // ~50% duty cycle
  ledcWrite(PWM_CHANNEL_L, 0);

  spreadStartTime = millis();
  isSpreading = true;
}

void highSpread() {
  Serial.println("Starting high spread (1000 RPM)");
  digitalWrite(MOTOR_EN_R, HIGH);
  digitalWrite(MOTOR_EN_L, HIGH);
  ledcWrite(PWM_CHANNEL_R, 255);  // 100% duty cycle
  ledcWrite(PWM_CHANNEL_L, 0);

  spreadStartTime = millis();
  isSpreading = true;
}

void stopMotor() {
  digitalWrite(MOTOR_EN_R, LOW);
  digitalWrite(MOTOR_EN_L, LOW);
  ledcWrite(PWM_CHANNEL_R, 0);
  ledcWrite(PWM_CHANNEL_L, 0);
  Serial.println("Motor stopped");
  isSpreading = false;
}

// Dispense feed function
void dispenseFeed(int grams, int type) {
  int servoAngle = 90; // Dispensing position
  int dispenseDuration = 0;

  if (type == 1) {
    dispenseDuration = 680;
  } else if (type == 2) {
    dispenseDuration = 500;
  } else {
    Serial.println("Invalid feed type.");
    return;
  }

  if (grams >= 100) {
    int num100g = grams / 100;
    for (int i = 0; i < num100g; i++) {
      Serial.printf("Dispensing 100g of feed type %d (%d of %d) - Duration: %d ms\n", type, i + 1, num100g, dispenseDuration);
      myservo.write(servoAngle);
      delay(dispenseDuration);
      myservo.write(180);
      delay(1000);
    }
    stopMotor();
    return;
  }

  Serial.println("Invalid weight specified.");
}

// Process manual serial commands
void processCommand(String input) {
  int firstSpaceIndex = input.indexOf(' ');
  int secondSpaceIndex = input.indexOf(' ', firstSpaceIndex + 1);

  if (firstSpaceIndex == -1 || secondSpaceIndex == -1) {
    Serial.println("Invalid format! Use: <grams> <mode(HIGH/CONTROL)> <type(1/2)>");
    return;
  }

  int grams = input.substring(0, firstSpaceIndex).toInt();
  String mode = input.substring(firstSpaceIndex + 1, secondSpaceIndex);
  int type = input.substring(secondSpaceIndex + 1).toInt();

  mode.toUpperCase();

  if (grams <= 0) {
    Serial.println("Grams must be positive.");
    return;
  }
  if (type < 1 || type > 2) {
    Serial.println("Type must be 1 or 2.");
    return;
  }

  Serial.println("Starting motor for 5 seconds...");
  highSpread();
  delay(5000);
  Serial.println("5 seconds complete. Continuing with specified mode...");

  if (mode == "HIGH") {
    highSpread();
    dispenseFeed(grams, type);
  } else if (mode == "CONTROL") {
    controlledSpread();
    dispenseFeed(grams, type);
  } else {
    Serial.println("Invalid mode! Use HIGH or CONTROL");
    stopMotor();
  }
}

// Poll server for feeding commands and execute
void checkFeedingCommand() {
  if (feedingInProgress) return; // Avoid overlapping commands

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(feedCommandUrl);

    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) {
      String payload = http.getString();
      Serial.println("Feeding command received:");
      Serial.println(payload);

      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        int grams = doc["grams"] | 0;
        const char* mode = doc["mode"] | "";
        int type = doc["type"] | 0;

        Serial.printf("grams: %d, mode: %s, type: %d\n", grams, mode, type);

        if (grams > 0 && (strcmp(mode, "HIGH") == 0 || strcmp(mode, "CONTROL") == 0) && (type == 1 || type == 2)) {
          feedingInProgress = true;

          // Start motor for 5 seconds at high speed
          Serial.println("Starting motor for 5 seconds (feeding command)...");
          highSpread();
          delay(5000);

          Serial.println("5 seconds complete. Continuing with specified mode...");
          if (strcmp(mode, "HIGH") == 0) {
            highSpread();
            dispenseFeed(grams, type);
          } else {
            controlledSpread();
            dispenseFeed(grams, type);
          }

          feedingInProgress = false;
        } else {
          Serial.println("Invalid feeding command parameters.");
        }
      } else {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.printf("Error getting feeding command: %d\n", httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi not connected - cannot check feeding command");
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");
  delay(2000);
  lcd.clear();

  // Initialize DS18B20
  sensors.begin();

  // Set ADC resolution (12-bit)
  analogReadResolution(12);

  // Initialize servo
  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50); // 50Hz standard servo frequency
  myservo.attach(servoPin, 500, 2500); // min/max pulse width in microseconds
  myservo.write(180); // Closed position

  // Initialize motor pins
  pinMode(MOTOR_EN_R, OUTPUT);
  pinMode(MOTOR_EN_L, OUTPUT);

  ledcAttach(MOTOR_PWM_R, PWM_CHANNEL_R, PWM_RESOLUTION);
  ledcAttach(MOTOR_PWM_L, PWM_CHANNEL_L, PWM_RESOLUTION);

  // Motors off initially
  digitalWrite(MOTOR_EN_R, LOW);
  digitalWrite(MOTOR_EN_L, LOW);
  ledcWrite(PWM_CHANNEL_R, 0);
  ledcWrite(PWM_CHANNEL_L, 0);

  // Connect to WiFi
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...  ");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected ");
  lcd.setCursor(0, 1);
  lcd.print("IP:");
  lcd.print(WiFi.localIP());
  delay(3000);
  lcd.clear();

  // Seed random number generator
  randomSeed(analogRead(0));

  Serial.println("System Ready.");
  Serial.println("Enter commands in format: <grams> <mode(HIGH/CONTROL)> <type(1/2)>");
}

void loop() {
  // === Sensor readings ===
  pH_value = readPH();           // replaced simulated pH with actual sensor reading
  ammoniaLevel = readAmmonia(); // replaced simulated ammonia with actual sensor reading

  int turbidity_analog = analogRead(turbidity_Pin);
  turbidity_voltage = turbidity_analog * (3.3 / 4095.0);
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);

  // === Serial Output and Fuzzy Logic ===
  Serial.println("====== Water Quality ======");

  Serial.print("pH: ");
  Serial.print(pH_value, 2);
  Serial.print(" mg/L | Ammonia: ");
  Serial.print(ammoniaLevel, 3);
  Serial.print(" ppm | Turbidity Voltage: ");
  Serial.print(turbidity_voltage, 2);
  Serial.print(" V | Temp: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  // === Fuzzy Logic (pH) ===
  if (pH_value >= 6.5 && pH_value <= 8.5) {
    int range = map(pH_value * 10, 65, 85, 1, 3);
    switch (range) {
      case 1: Serial.print("      LOWER "); break;
      case 2: Serial.print("      NORMAL "); break;
      case 3: Serial.print("      HIGHER "); break;
    }
    Serial.println("GOOD");
  }
  else if ((pH_value >= 5.5 && pH_value <= 6.4) || (pH_value >= 8.6 && pH_value <= 9.0)) {
    if (pH_value >= 5.5 && pH_value <= 6.4) {
      int range = map(pH_value * 10, 55, 64, 1, 3);
      switch (range) {
        case 1: Serial.print("      LOWER "); break;
        case 2: Serial.print("      LOWLY "); break;
        case 3: Serial.print("      NORMAL "); break;
      }
    }
    else if (pH_value >= 8.6 && pH_value <= 9.0) {
      int range = map(pH_value * 10, 86, 90, 1, 3);
      switch (range) {
        case 1: Serial.print("      NORMAL "); break;
        case 2: Serial.print("      HIGHLY "); break;
        case 3: Serial.print("      HIGHER "); break;
      }
    }
    Serial.println("MODERATE");
  }
  else if ((pH_value >= 4.5 && pH_value <= 5.4) || (pH_value >= 9.1 && pH_value <= 10.5)) {
    if (pH_value >= 4.5 && pH_value <= 5.4) {
      int range = map(pH_value * 10, 45, 54, 1, 3);
      switch (range) {
        case 1: Serial.print("      LOWER "); break;
        case 2: Serial.print("      LOWLY "); break;
        case 3: Serial.print("      NORMAL "); break;
      }
    }
    else if (pH_value >= 9.1 && pH_value <= 10.5) {
      int range = map(pH_value * 10, 91, 105, 1, 3);
      switch (range) {
        case 1: Serial.print("      NORMAL "); break;
        case 2: Serial.print("      HIGHLY "); break;
        case 3: Serial.print("      HIGHER "); break;
      }
    }
    Serial.println("POOR");
  }
  else if (pH_value < 4.5 || pH_value > 10.5) {
    if (pH_value < 4.5) Serial.print("      LOWER ");
    if (pH_value > 10.5) Serial.print("      HIGHER ");
    Serial.println("BAD");
  }

  Serial.print("Turbidity Voltage: ");
  Serial.print(turbidity_voltage, 2);
  Serial.println(" V");

  // === Fuzzy Logic (Turbidity) ===
  if (turbidity_voltage > 0 && turbidity_voltage <= 1.5) {
    int range = map(turbidity_voltage * 10, 0, 15, 1, 3);
    switch (range) {
      case 1: Serial.print("      LOWER "); break;
      case 2: Serial.print("      NORMAL "); break;
      case 3: Serial.print("      HIGHER "); break;
    }
    Serial.println("GOOD");
  }
  else if (turbidity_voltage >= 1.0 && turbidity_voltage <= 3.0) {
    int range = map(turbidity_voltage * 10, 10, 30, 1, 3);
    switch (range) {
      case 1: Serial.print("      LOWER "); break;
      case 2: Serial.print("      NORMAL "); break;
      case 3: Serial.print("      HIGHER "); break;
    }
    Serial.println("MODERATE");
  }
  else if (turbidity_voltage >= 2.1 && turbidity_voltage <= 5.0) {
    int range = map(turbidity_voltage * 10, 21, 50, 1, 3);
    switch (range) {
      case 1: Serial.print("      LOWER "); break;
      case 2: Serial.print("      NORMAL "); break;
      case 3: Serial.print("      HIGHER "); break;
    }
    Serial.println("POOR");
  }
  else if (turbidity_voltage > 3.0) {
    Serial.println("BAD");
  }

  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  // === Fuzzy Logic (Temperature) ===
  if (temperature >= 26 && temperature <= 29) {
    int range = map(temperature * 10, 260, 290, 1, 3);
    switch (range) {
      case 1: Serial.print("      LOWER "); break;
      case 2: Serial.print("      NORMAL "); break;
      case 3: Serial.print("      HIGHER "); break;
    }
    Serial.println("GOOD");
  }
  else if ((temperature >= 22 && temperature <= 26) || (temperature >= 29 && temperature <= 31)) {
    if (temperature >= 22 && temperature <= 26) {
      int range = map(temperature * 10, 220, 260, 1, 3);
      switch (range) {
        case 1: Serial.print("      LOWER "); break;
        case 2: Serial.print("      LOWLY "); break;
        case 3: Serial.print("      NORMAL "); break;
      }
    }
    else if (temperature >= 29 && temperature <= 31) {
      int range = map
