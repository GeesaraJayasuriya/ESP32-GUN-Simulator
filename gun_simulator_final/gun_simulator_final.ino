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
float potentiometerValue = 0.0;

const int SOLENOID_PIN = 5; // Change this to the pin number where your solenoid is connected
const int LASER_PIN = 18; // Change this to the pin number where your laser sensor is connected

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

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
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
    float potentiometerValue = root["potentiometer"];
    float gyro_x = root["gyro_x"];
    float gyro_y = root["gyro_y"];
    float gyro_z = root["gyro_z"];
    int   counter =root["counter"];
    bool  load_unload = root["load_unload"];
    
    // Send response
    String response = "{\"forceSensor1\":" + String(forceSensor1Value) + ",\"forceSensor2\":" + String(forceSensor2Value) + ",\"forceSensor3\":" + String(forceSensor3Value) + ",\"forceSensor4\":" + String(forceSensor4Value)+ ",\"gyro_x\":" + String(gyro_x) + ",\"gyro_y\":" + String(gyro_y) + ",\"gyro_z\":" + String(gyro_z)+",\"counter\":" + String(counter)+",\"load_unload\":" + String(load_unload) + ",\"potentiometer\":" + String(potentiometerValue) + "}";
    request->send(200, "application/json", response);

    file.close();
  });

  // Start server
  server.begin();
}

void loop() {
  unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        int reedSwitchState1 = digitalRead(reedSwitchPin1);
        int reedSwitchState2 = digitalRead(reedSwitchPin2);
        // Read force sensor values
        for (int i = 0; i < 4; i++) {
          forceSensorValues[i] = analogRead(32+i);
        }
          sensors_event_t accel, gyro,temp;
          mpu.getEvent(&accel, &gyro,&temp);
        
        // Read potentiometer value
        potentiometerValue = analogRead(A0) * (3.3 / 4095.0);
        
        // Convert analog values to voltage (assuming 3.3V reference voltage)
        float forceSensor1Voltage = forceSensorValues[0];
        float forceSensor2Voltage = forceSensorValues[1];
        float forceSensor3Voltage = forceSensorValues[2];
        float forceSensor4Voltage = forceSensorValues[3];
    
        float accel_acceleration_x = accel.acceleration.x;
        float accel_acceleration_y = accel.acceleration.y;
        float accel_acceleration_z = accel.acceleration.z;
    
    
         // Write data to file
        File file = SPIFFS.open("/data/data.json", FILE_WRITE);
        if (!file) {
          Serial.println("Failed to open file for writing");
          return;
        }
    
        StaticJsonDocument<200> doc;
         if (reedSwitchState1 == LOW) {
            // check if the switch has changed state
            if (reedSwitchState2 == HIGH && !isSwitchClosed) {
                  // switch has gone from LOW to HIGH, increment counter
                      counter++;
                      isSwitchClosed = true;
            } else if (reedSwitchState2 == LOW && isSwitchClosed) {
                  // switch has gone from HIGH to LOW, reset switch closed flag
                      isSwitchClosed = false;
             }
            doc["load_unload"] = reedSwitchState1;
            doc["forceSensor1"] = forceSensor1Voltage;
            doc["forceSensor2"] = forceSensor2Voltage;
            doc["forceSensor3"] = forceSensor3Voltage;
            doc["forceSensor4"] = forceSensor4Voltage;
            doc["potentiometer"] =potentiometerValue;
            doc["gyro_x"] = accel_acceleration_x;
            doc["gyro_y"] = accel_acceleration_y;
            doc["gyro_z"] = accel_acceleration_z;
            doc["counter"] = counter;
            
            if (potentiometerValue >= 2.0) {
              processPotValue(potentiometerValue);
            }
            
         }else{
            doc["load_unload"] = HIGH;
            doc["forceSensor1"] = 0.0;
            doc["forceSensor2"] = 0.0;
            doc["forceSensor3"] = 0.0;
            doc["forceSensor4"] = 0.0;
            doc["potentiometer"] =0.0;
            doc["gyro_x"] = 0.0;
            doc["gyro_y"] = 0.0;
            doc["gyro_z"] = 0.0;
            doc["counter"] = 0;
          
         }
        serializeJson(doc, file);
        file.close();
      }
}

void processPotValue(int value){
  if(value){
    pinMode(SOLENOID_PIN,HIGH);
    delay(1000);
    pinMode(SOLENOID_PIN,LOW);
    digitalWrite(LASER_PIN, HIGH);    // Output a HIGH value to the laser pin
    delay(1000);                      // Wait for 1 second
    digitalWrite(LASER_PIN, LOW);
    }
}
