#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

#define MPU_ADDR 0x68 // MPU6050 I2C address

Adafruit_MPU6050 mpu;

const char* ssid = "Geesaramax";
const char* password = "eket9508";

AsyncWebServer server(80);

float forceSensorValues[4];

const int SOLENOID_PIN = 5; // Change this to the pin number where your solenoid is connected
const int LASER_PIN = 18; // Change this to the pin number where your laser sensor is connected
const int buttonPin = 19; // Change this to the pin number where the button is connected

unsigned long previousMillis = 0;
const long interval = 1000;  // Interval to read sensor value in milliseconds

const int reedSwitchPin1 = 2;
const int reedSwitchPin2 = 4;
int counter = 0;
bool isSwitchClosed = false;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.begin(MPU_ADDR);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(reedSwitchPin1, INPUT_PULLUP);
  pinMode(reedSwitchPin2, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  // Set up web server
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest * request) {
    // Read data from file
    File file = SPIFFS.open("/data/data.json", FILE_READ);
    if (!file) {
      Serial.println("Failed to open file for reading");
      request->send(500, "text/plain", "Failed to open file for reading");
      return;
    }

    StaticJsonDocument<400> doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
      Serial.println("Failed to parse file");
      request->send(500, "text/plain", "Failed to parse file");
      return;
    }

    JsonObject root = doc.as<JsonObject>();
    float forceSensor1Value = root["forceSensor1"];
    float forceSensor2Value = root["forceSensor2"];
    float forceSensor3Value = root["forceSensor3"];
    float forceSensor4Value = root["forceSensor4"];
    bool buttonPressed = root["buttonPressed"];
    float gyro_x = root["gyro_x"];
    float gyro_y = root["gyro_y"];
    float gyro_z = root["gyro_z"];
    int   counter = root["counter"];
    bool  load_unload = root["load_unload"];
    // Send response
    String response = "{\"forceSensor1\":" + String(forceSensor1Value) + ",\"forceSensor2\":" + String(forceSensor2Value) + ",\"forceSensor3\":" + String(forceSensor3Value) + ",\"forceSensor4\":" + String(forceSensor4Value) + ",\"gyro_x\":" + String(gyro_x) + ",\"gyro_y\":" + String(gyro_y) + ",\"gyro_z\":" + String(gyro_z) + ",\"counter\":" + String(counter) + ",\"load_unload\":" + String(load_unload) + ",\"buttonPressed\":" + String(buttonPressed) + "}";
    request->send(200, "application/json", response);
    file.close();
  });

  // Start server
  server.begin();
}
void loop() {
  static unsigned long previousMillis = 0;
  const unsigned long interval = 100;
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
    // Read sensor values
    int reedSwitchState1 = digitalRead(reedSwitchPin1);
    int reedSwitchState2 = digitalRead(reedSwitchPin2);
    int buttonState      = digitalRead(buttonPin);
    float forceSensorValues[4];
    sensors_event_t accel, gyro, temp;
    // Update switch counter
    static bool isSwitchClosed = false;
    static int counter = 0;
    // Check button press
    static bool isButtonPressed = false;
    if (reedSwitchState1 == LOW) {
      if (digitalRead(buttonPin) == HIGH && !isButtonPressed) {
        pinMode(SOLENOID_PIN, HIGH);
        delay(1000);
        pinMode(SOLENOID_PIN, LOW);
        digitalWrite(LASER_PIN, HIGH);    // Output a HIGH value to the laser pin
        delay(2000);                      // Wait for 1 second
        digitalWrite(LASER_PIN, LOW);
        isButtonPressed = true;
        if (reedSwitchState2 == HIGH && !isSwitchClosed) {
          counter++;
          isSwitchClosed = true;
        } else if (reedSwitchState2 == LOW && isSwitchClosed) {
          isSwitchClosed = false;
        } else {
          counter = 0;
          isSwitchClosed = false;
        }
        for (int i = 0; i < 4; i++) {
          forceSensorValues[i] = analogRead(32 + i) * (3.3 / 4095.0);
        }
        mpu.getEvent(&accel, &gyro, &temp);
      } else if (digitalRead(buttonPin) == LOW && isButtonPressed) {
        isButtonPressed = false;
      }
    } else {
      counter = 0;
      isSwitchClosed = false;
    }



    // Write data to file
    File file = SPIFFS.open("/data/data.json", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing");
      return;
    }
    StaticJsonDocument<200> doc;
    doc["load_unload"] = reedSwitchState1;
    doc["forceSensor1"] = forceSensorValues[0];
    doc["forceSensor2"] = forceSensorValues[1];
    doc["forceSensor3"] = forceSensorValues[2];
    doc["forceSensor4"] = forceSensorValues[3];
    doc["buttonPressed"] = buttonState;
    doc["gyro_x"] = accel.acceleration.x;
    doc["gyro_y"] = accel.acceleration.y;
    doc["gyro_z"] = accel.acceleration.z;
    doc["counter"] = counter;
    serializeJson(doc, file);
    file.close();
  }
}
