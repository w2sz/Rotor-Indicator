#include <Arduino.h>
#include "MPU9250.h"
#include "imu_eeprom.h"
#include <vector>

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// WIFI Setup
WebServer server(80); // Web server on port 80

const char *ssid = "ESP32-gyro";   // AP SSID
const char *password = "12345678"; // AP password

MPU9250 mpu;

void update_imu(void *pvParameters);
void print_result(void *pvParameters);
void updateBuffer(float *buffer, float value);
bool imu_calibration();
void handleRoot();

// float fl_yaw = 0.f;
// float fl_roll = 0.f;
// float fl_pitch = 0.f;
const int buffer_size = 5;
float window_yaw[buffer_size];
float window_roll[buffer_size];
float window_pitch[buffer_size];
int buffer_index = 0;

void setup()
{
  // Initialize buffers with zeros
  memset(window_yaw, 0, sizeof(window_yaw));
  memset(window_roll, 0, sizeof(window_roll));
  memset(window_pitch, 0, sizeof(window_pitch));

  // Set up ESP32 as an Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Created. Connect to the ESP32.");

  // Define the route for JSON data
  server.on("/", handleRoot);
  server.begin(); // Start the web server

#if defined(ESP_PLATFORM) || defined(ESP8266)
  EEPROM.begin(0x80);
#endif
  Serial.begin(115200);
  Wire.begin();
  mpu.setMagneticDeclination(-13.5);
  mpu.setup(0x68);
  mpu.setFilterIterations(20);
  Serial.println(isCalibrated());
  printCalibration();
  Serial.println("Press n to calibrate, y to load calibrated data");

  unsigned long lastSerialTime = millis();    // Start the timer
  const unsigned long timeoutDuration = 5000; // 5 seconds
  bool timeoutOccurred = false;               // Flag to track if timeout occurred

  // Wait for serial input or timeout after 5 seconds
  while (!Serial.available() > 0)
  {
    if (millis() - lastSerialTime > timeoutDuration)
    {
      timeoutOccurred = true; // Set flag when timeout occurs
      break;                  // Break out of the loop and continue to the end
    }
  }

  // Check if timeout occurred
  if (timeoutOccurred)
  {
    loadCalibration(); // Load calibration automatically after 5 seconds
  }
  else
  {
    // After serial input is available, process it
    if (Serial.read() == 'y' && isCalibrated())
    {
      loadCalibration(); // Load calibration if 'y' is received and isCalibrated() returns true
    }
    else
    {
      if (!imu_calibration())
      {
        Serial.println("Calibration failed");
        ESP.restart(); // Restart if calibration fails
      }
    }
  }

  xTaskCreatePinnedToCore(
      update_imu,   // Task function
      "update_imu", // Task name
      2048,         // Stack size (in words)
      NULL,         // Task input parameter
      1,            // Task priority
      NULL,         // Task handle
      1             // Core 1
  );

  xTaskCreatePinnedToCore(
      print_result,   // Task function
      "print_result", // Task name
      2048,           // Stack size (in words)
      NULL,           // Task input parameter
      1,              // Task priority
      NULL,           // Task handle
      1               // Core 1
  );

  // xTaskCreatePinnedToCore(
  //     handleRoot,   // Task function
  //     "handleRoot", // Task name
  //     2048,         // Stack size (in words)
  //     NULL,         // Task input parameter
  //     1,            // Task priority
  //     NULL,         // Task handle
  //     1             // Core 1
  // );
}

void loop()
{
  server.handleClient();
}
void update_imu(void *pvParameters)
{
  const TickType_t xDelay = 10 / portTICK_PERIOD_MS;
  while (1)
  {
    mpu.update();
    updateBuffer(window_yaw, mpu.getYaw());     // Update yaw buffer
    updateBuffer(window_roll, mpu.getRoll());   // Update roll buffer
    updateBuffer(window_pitch, mpu.getPitch()); // Update pitch buffer
    vTaskDelay(xDelay);
  }
}

void updateBuffer(float *buffer, float value)
{
  buffer[buffer_index] = value;                    // Insert the new value at the current index
  buffer_index = (buffer_index + 1) % buffer_size; // Move to the next position, wrap around if necessary
}

float bufferReadOut(float *buffer)
{
  float sum = 0.0f;
  for (int i = 0; i < buffer_size; ++i)
  {
    sum += buffer[i]; // Access each element of the buffer
  }
  return sum / buffer_size; // Return the average
}

void print_result(void *pvParameters)
{
  const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

  while (1)
  {

    // Serial.print(mpu.getYaw());
    // Serial.print(",");
    // Serial.print(mpu.getRoll());
    // Serial.print(",");
    // Serial.println(mpu.getPitch());

    Serial.print(bufferReadOut(window_yaw));
    Serial.print(",");
    Serial.print(bufferReadOut(window_roll));
    Serial.print(",");
    Serial.println(bufferReadOut(window_pitch));

    vTaskDelay(xDelay);
  }
}

void handleRoot()
{
  float yaw = bufferReadOut(window_yaw);
  float roll = bufferReadOut(window_roll);
  float pitch = bufferReadOut(window_pitch);

  // Create JSON object
  StaticJsonDocument<200> doc;
  doc["yaw"] = yaw;
  doc["roll"] = roll;
  doc["pitch"] = pitch;

  String jsonString;
  serializeJson(doc, jsonString);

  server.send(200, "application/json", jsonString); // Send the JSON response
}

bool imu_calibration()
{
  Serial.println("Start calibrate gyro");
  mpu.calibrateAccelGyro();
  Serial.println("Gyro calibration complete");
  Serial.println("Start calibrate mag");
  mpu.calibrateMag();
  Serial.println("Mag calibration complete");
  Serial.println("Press y to save calibrated data");
  while (!Serial.available())
    ;
  if (Serial.read() == 'y')
  {
    saveCalibration();
    Serial.println("Calibration date is saved into EEPROM");
    return true;
  }
  else if (Serial.read() == 'n')
  {
    return false;
  }
}