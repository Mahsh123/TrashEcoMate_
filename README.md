# TrashEcoMate

A comprehensive waste management system comprising:
- **TrashEcoBot**: A ROS-based autonomous robot for waste collection, built on Raspberry Pi.
- **TrashEcoBin**: An ESP32-powered bin monitoring system that tracks waste levels.
- **TrashEcoMate_app**: A Flutter app for real-time monitoring of bin status.

The system uses Firebase for data synchronization, enabling efficient waste collection by monitoring bin levels and controlling the robot's navigation with obstacle avoidance using an ultrasonic sensor mounted on a servo motor. Lightweight machine learning (Q-learning) enhances navigation for smarter obstacle avoidance.

## Project Structure
```
TrashEcoMate/
├── TrashEcoBot/
│   ├── trashEcoBot_ws/
│   │   ├── src/
│   │   │   └── trashecomate/
│   │   │       ├── src/
│   │   │       │   ├── robot_movement.py  # Robot navigation with servo and Q-learning
│   │   │       │   ├── firebase_listener.py  # Publishes bin levels to ROS
│   │   │       ├── launch/
│   │   │       │   ├── trashecomate_launch.launch  # Launch file for ROS nodes
│   │   │       ├── CMakeLists.txt
│   │   │       ├── package.xml
├── TrashEcoBin/
│   ├── bin_sensor.ino  # ESP32 code for bin monitoring
│   ├── README.md  # Instructions for ESP32 setup
├── TrashEcoMate_app/
│   # (Flutter app code to be added)
├── .gitignore
└── README.md
```

## Prerequisites

- **Hardware**:
  - Raspberry Pi 3/4 (for TrashEcoBot)
  - ESP32 (for TrashEcoBin)
  - Servo motor and ultrasonic sensor (HC-SR04) for robot navigation
  - L293D motor driver and DC motors for robot movement
- **Software**:
  - ROS Noetic (on Raspberry Pi)
  - Arduino IDE (for ESP32)
  - Python 3 (with `rospy`, `RPi.GPIO`, `firebase_admin`, `numpy`)
  - Firebase account for Realtime Database
- **Dependencies**:
  ```bash
  sudo apt update
  sudo apt install python3-pip python3-empy
  pip3 install firebase_admin numpy
  ```


## Setup
### 1. Clone the Repository
```bash

git clone https://github.com/your-username/TrashEcoMate.git
cd TrashEcoMate
```
### 2. Configure Firebase
Create a Firebase project and enable Realtime Database.
Download the serviceAccountKey.json file and place it in /home/your-username/waste_management_robot/.
Set environment variables:
```bash

export FIREBASE_CREDENTIALS="/home/your-username/waste_management_robot/serviceAccountKey.json"
export FIREBASE_URL="https://firebasedatabase.app/"
```
### 3. Build TrashEcoBot
Navigate to the ROS workspace:
```bash
cd TrashEcoBot/trashEcoBot_ws
catkin_make
source devel/setup.bash
```
Ensure the servo motor is connected to GPIO 13 (Physical Pin 33) on the Raspberry Pi, with the ultrasonic sensor mounted on top.

### 4. Upload TrashEcoBin Code
Open TrashEcoBin/bin_sensor.ino in the Arduino IDE.
Configure Firebase credentials in a config.h file (not tracked by Git):
```cpp
#define FIREBASE_URL "https://trashecomate-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_API_KEY "your-api-key"
```
Upload the code to the ESP32.
### 5. (Optional) Set Up TrashEcoMate_app
(Instructions for Flutter app setup to be added once the app is integrated.)
