#include <WiFi.h>
#include <WiFiUdp.h>
#include "MPU9250.h"
#include <Wire.h> // Needed for I2C to read IMU
#include <ArduinoJson.h> // Compatible amb versió 7.4.2

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

// MPU-9250 object
MPU9250 mpu;

// Orientation data
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;

// Torque variables (values received from Servomotors module) ---
float Torque_roll1 = 0.0;
float Torque_pitch = 0.0;
float Torque_yaw = 0.0;

// Small buffer for UDP reads
char udpBuffer[512];

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

void updateOrientation() {
  if (mpu.update()) {
    Gri_yaw = -mpu.getYaw();
    Gri_pitch = -mpu.getPitch();
    Gri_roll = mpu.getRoll();
  }
  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);
}

void sendOrientationUDP() {
  // Using ArduinoJson - create an appropriately sized document
  StaticJsonDocument<256> doc;
  doc["device"] = deviceId;
  doc["roll"] = Gri_roll;
  doc["pitch"] = Gri_pitch;
  doc["yaw"] = Gri_yaw;
  doc["s1"] = s1Status;
  doc["s2"] = s2Status;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  // Send to ESP32 Servos
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}

// Receive torques (reads any incoming UDP packet and tries JSON or CSV) ---
void receiveTorquesUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return; // nothing to read

  // read into buffer (ensure null-terminated)
  int len = udp.read(udpBuffer, sizeof(udpBuffer) - 1);
  if (len <= 0) return;
  udpBuffer[len] = '\0';

  Serial.print("Received UDP packet: ");
  Serial.println(udpBuffer);

  bool parsed = false;

  // Try parse as JSON first
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, udpBuffer);
  if (!err) {
    // Try several common key names to be robust
    if (doc.containsKey("Torque_roll1")) {
      Torque_roll1 = doc["Torque_roll1"].as<float>();
    } else if (doc.containsKey("torque_roll")) {
      Torque_roll1 = doc["torque_roll"].as<float>();
    } else if (doc.containsKey("roll_torque")) {
      Torque_roll1 = doc["roll_torque"].as<float>();
    }

    if (doc.containsKey("Torque_pitch")) {
      Torque_pitch = doc["Torque_pitch"].as<float>();
    } else if (doc.containsKey("torque_pitch")) {
      Torque_pitch = doc["torque_pitch"].as<float>();
    } else if (doc.containsKey("pitch_torque")) {
      Torque_pitch = doc["pitch_torque"].as<float>();
    }

    if (doc.containsKey("Torque_yaw")) {
      Torque_yaw = doc["Torque_yaw"].as<float>();
    } else if (doc.containsKey("torque_yaw")) {
      Torque_yaw = doc["torque_yaw"].as<float>();
    } else if (doc.containsKey("yaw_torque")) {
      Torque_yaw = doc["yaw_torque"].as<float>();
    }

    parsed = true;
  }

  // If JSON parse failed, try a simple CSV or whitespace-separated format: "r,p,y" or "r p y"
  if (!parsed) {
    // copy into a temp string for strtok parsing
    char tmp[512];
    strncpy(tmp, udpBuffer, sizeof(tmp));
    tmp[sizeof(tmp)-1] = '\0';

    // replace commas with spaces to make tokenizing easier
    for (int i = 0; tmp[i] != '\0'; ++i) {
      if (tmp[i] == ',') tmp[i] = ' ';
    }

    char *token = strtok(tmp, " \t\r\n");
    float vals[3] = {0.0, 0.0, 0.0};
    int idx = 0;
    while (token != NULL && idx < 3) {
      vals[idx++] = atof(token);
      token = strtok(NULL, " \t\r\n");
    }
    if (idx >= 1) Torque_roll1 = vals[0];
    if (idx >= 2) Torque_pitch = vals[1];
    if (idx >= 3) Torque_yaw = vals[2];
  }

  // Print parsed torque values for debugging
  Serial.print("Torque_roll1: "); Serial.print(Torque_roll1);
  Serial.print("  Torque_pitch: "); Serial.print(Torque_pitch);
  Serial.print("  Torque_yaw: "); Serial.println(Torque_yaw);

  // Vibration motor control based on torque values ---
  float totalTorque = Torque_roll1 + Torque_pitch + Torque_yaw;
  // Convert torque to PWM value (0-255). Adjust the 2.5 scaling factor as needed.
  int vibrationValue = constrain((int)(totalTorque * 2.5f), 0, 255);
  ledcWrite(0, vibrationValue); // Set the PWM value for the vibration motor
  Serial.print("Vibration motor value: ");
  Serial.println(vibrationValue);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {
    while (1) {
      Serial.println("MPU connection failed.");
      delay(5000);
    }
  }
  Serial.println("MPU connected");
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);

  // Configure PWM for the vibration motor (channel 0) ---
  // Channel 0, frequency 5kHz, resolution 8 bits
  ledcSetup(0, 5000, 8);
  // Attach the vibration motor pin to channel 0
  ledcAttachPin(vibrationPin, 0);
  // Ensure vibration motor is off at start
  ledcWrite(0, 0);
}

void loop() {
  updateOrientation();

  // Check for incoming torque packets and react (vibration) ---
  receiveTorquesUDP();

  sendOrientationUDP();
  delay(10);
}
