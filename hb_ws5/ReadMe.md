# Welcome to eYRC Holo Battalion Theme 2025-26

---

Holonomic Robot Control using ROS 2 and ESP32 (MQTT)

This project implements a 3-wheeled holonomic mobile robot controlled using ROS 2 on a host machine and an ESP32 microcontroller over MQTT communication.


##Project Structure
--- 
.
├── esp_code/
│   └── mqtt_client.ino
│
└── hb_control/src
    ├── HB_1060_attach_service.py
    ├── HB_1060_holonomic_perception.py
    └── HB_1060_mutiholonomic_controller.py

└── hb_control/srv 
     └── Attach.srv


###ESP32 Code
esp_code/mqtt_client.ino

###Purpose:
Runs on the ESP32 and acts as an MQTT client that:
Connects to a Wi-Fi network
Subscribes to motor command topics
Parses velocity or wheel commands from ROS 2
Drives motors accordingly

###Key Features:
MQTT broker connection
Topic-based motor control
Command parsing (PWM / RPM / wheel velocity)
Designed for real-time robot motion control

###Typical Topics:
bot_cmd/ – Receives motor commands
esp/sensor/1 – Publishes 
esp/sol/1 -Receives solenoid commands

##ROS 2 Package (hb_control)
hb_control/src

###HB_1060_holonomic_perception.py

####Purpose:
Handles robot pose and perception logic.

####Functionality:

Processes camera feed.
Publishes robot pose arrays by detecting aruco markers.
Useful for visualization and feedback control

####Publishes:

Pose2D
Poses2D

Topic:
/bot_pose
/crate_pose


##HB_1060_mutiholonomic_controller.py

####Purpose:
Implements inverse kinematics for a 3-wheel holonomic drive of 3 botsand publishes wheel-speeds
to esp32 through MQTT protocol.

####Functionality:
Process the poses the bots and crates and allocates the crates to boxes using hungarian algorithm.
Implements PID by processing pose data sent through topics /bot_pose, /crate_pose.
Uses ORCA algorithm for bots collision avoidance.
Converts linear and angular velocity into individual wheel velocities
Enforces velocity limits
Normalizes wheel speeds 
Publishes wheel speeds through MQTT protocols.

####Topic:
/bot_pose
/crate_pose
/bot_cmd

##HB_1060_attach_service.py

####Purpose:
ROS 2 service node to ON or OFF solenoid.

####Functionality:
Turns ON/OFF solenoid via service calls
Useful for picking up crates.
Takes bot number and bool values as input.

####Service Type:
from hb_control.srv import Attach 

####Topic: 
/attach


###ROS 2 Package (hb_control)
hb_control/srv

##Attach.srv

####Purpose:
Custom ros2 service.

####Functionality:
Takes bot number and bool values as input.
Gives success, string msg as output.

####Service Type:
from hb_control.srv import Attach



Data Flow Overview::

camera_feed
   ↓
holonomic_perception
   ↓ 
multiholonomic_controller
   ↓  (wheel velocities)
attach_service.py(when near crate)
   ↓
ESP32 (mqtt_client.ino)
   ↓
Motors





