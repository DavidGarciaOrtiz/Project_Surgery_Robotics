#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h>
#include <ArduinoJson.h>  // No cal .hpp, la versió 7 manté compatibilitat amb .h
#include <IMU_RoboticsUB.h>   // Nom de la llibreria custom

// Device ID
const char *deviceId = "G4_Gri";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// Vibration motor settings
const int vibrationPin = 23; // Pin for the vibration motor

// Botons
const int PIN_S1 = 14;
const int PIN_S2 = 27;
int s1Status = HIGH;
int s2Status = HIGH;

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 43); // IP of receiver ESP32
IPAddress receiverComputerIP(192, 168, 1, 45); // IP of PC
const int udpPort = 12345;
WiFiUDP udp;

// IMU object
IMU imu;

// Orientation data
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;

// Torque data received from G4_Servos
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;

// Function to connect to Wi-Fi
void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}

// Function to read IMU data and update orientation
void updateOrientation() {
  // Llegeix FIFO del DMP i actualitza càlculs interns
  imu.ReadSensor();
  // Obté els angles (roll, pitch, yaw) via GetRPW()
  float* rpw = imu.GetRPW();
  Gri_roll  = rpw[0];
  Gri_pitch = rpw[1];
  Gri_yaw   = rpw[2];
  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);
}

// Function to send orientation data via UDP
void sendOrientationUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["roll"] = Gri_roll;
  doc["pitch"] = Gri_pitch;
  doc["yaw"] = Gri_yaw;
  doc["s1"] = s1Status;
  doc["s2"] = s2Status;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  // Send to ESP32 Servos
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

// Function to receive torque data via UDP
void receiveTorquesUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    byte packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = '\0';

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char* device = doc["device"];
      if (strcmp(device, "G4_Servos") == 0) {
        Torque_roll1 = doc["Torque_roll1"];
        Torque_roll2 = doc["Torque_roll2"];
        Torque_pitch = doc["Torque_pitch"];
        Torque_yaw   = doc["Torque_yaw"];

        Serial.print("Received torques → ");
        Serial.print("Roll1: "); Serial.print(Torque_roll1, 3);
        Serial.print(" | Roll2: "); Serial.print(Torque_roll2, 3);
        Serial.print(" | Pitch: "); Serial.print(Torque_pitch, 3);
        Serial.print(" | Yaw: "); Serial.println(Torque_yaw, 3);

        // Vibration motor control based on torque values
        float totalTorque = Torque_roll1 + Torque_pitch + Torque_yaw;
        // Convert torque to PWM value (0-255)
        int vibrationValue = constrain(totalTorque * 2.5, 0, 255); // Adjust the scaling factor as needed
        ledcWrite(0, vibrationValue); // Set the PWM value for the vibration motor
        Serial.print("Vibration motor value: ");
        Serial.println(vibrationValue); 
      }
    }
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  // Inicialitza IMU (amb DMP)
  imu.Install();

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized.");

  // Configure PWM for the vibration motor (channel 0)
  ledcSetup(0, 5000, 8); // Channel 0, frequency 5kHz, resolution 8 bits
  ledcAttachPin(vibrationPin, 0); // Attach the vibration motor to channel 0
}

// Main loop
void loop() {
  updateOrientation();    // Read IMU and update orientation
  sendOrientationUDP();   // Send orientation data to servos & PC via UDP
  receiveTorquesUDP();    // Receive torque data from servos via UDP
  delay(10);
}
