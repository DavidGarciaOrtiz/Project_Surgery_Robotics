# **Surgery Robotics Project setup**

The objectives of this section are:
- Setup the project in student's github
- Review the needed tools
- Create a simple ESP32 Blink project with PlatformIO and VS Code

## **1. Setup the project**

You have to select a local folder to clone the Project repository. 
- Open Visual Studio Code on this folder

### First time
- Open a git terminal and clone the forked github project from Director's repository.
- add credential information from your github account
  ````shell
  git config --global user.email "mail@alumnes.ub.edu" 
  git config --global user.name "your github username"
  ````
- You will have to add the `.vscode/settings.json` in your local repository

### Next times
- before working on your project you have to update your local repository with last changes on remote github repository:
  ````shell
  git pull
  ````
- Now you can work on the project locally
- to update your changes:
  ````shell
  git add .
  git commit -m "your message"
  git push
  ````
- You can check the changes in your github repository

## **2. Review the needed tools**

You will need to use a new tool to program the ESP32 microcontroller.:
- Visual Studio Code with extension "PlatformIO IDE" (PlatformIO)


## **3. Create a simple ESP32 Blink project**

This project demonstrates how to blink an LED using an ESP32 board programmed with PlatformIO and the Arduino framework inside Visual Studio Code.

## 🧰 Requirements

- ESP32 DevKit v1
- USB cable
- Visual Studio Code
- PlatformIO IDE Extension

## 🚀 Create the Blink Project

- Click the alien icon (PlatformIO Home)
- Click on Projects & Configuration
- Click `New Project`
- Name the project: `ESP32Test_Blink`
- Select board: `Espressif ESP32 Dev Module`
- Framework: `Arduino`
- Click `Finish`

PlatformIO will create the project structure and download necessary tools.
The final structure will be like this:

```
ESP32Test_Blink/
├── src/
│   └── main.cpp
├── lib
└── platformio.ini
```

To open an existing project, click `Open Project` in PlatformIO Home and select the project folder (from the list of existing projects or you have to search it from your local PC). The selected project will appear in new folder on root path of VScode.

If the platform is the same, you can make a copy of the project folder and rename it to your new project name.

## 🧾 Source Code: `src/main.cpp`

For this speciffic exemple, replace the content of `src/main.cpp` with the following code:
```cpp
#include <Arduino.h>

const int ledPin = 2; // GPIO2, sovint connectat a un LED integrat

void setup() {
  Serial.begin(115200); // Inicialitza el port sèrie
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  Serial.println("Led switched ON");
  delay(1000);

  digitalWrite(ledPin, LOW);
  Serial.println("Led switched OFF");
  delay(1000);
}
```

## ⚙️ Configuration File: `platformio.ini`
Here we will add the "monitor_speed" option to set the serial monitor baud rate to 115200.

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

## ⚙️ Add libraries

For this exemple you do not need any library, but in a general project to add libraries:

- Go to the PlatformIO Home → Libraries tab.
- Search for the library you want.
- Click "Add to Project" and select your project.

## 🚀 Upload and Monitor

To upload your code to the ESP32 board and monitor the serial output:

1. Connect your ESP32 board to your computer via USB.
2. In PlatformIO, click the "Upload" button (right arrow icon) in the bottom toolbar.
3. After the upload is complete, click the "Monitor" button (magnifying glass icon) to open the serial monitor.
4. You should see the serial output from your ESP32 board.
5. The onboard LED should blink on and off every second.