# Multi-Robot Autonomous Sorting System

This repository contains a ROS2-based simulation featuring three holonomic robots—**Glyco**, **Crystal**, and **Frostbite**. The system is designed to demonstrate autonomous task allocation, path planning, and color-based object sorting within a warehouse environment.

## 📁 Project Structure

The environment consists of a central map with specific functional areas:

* **Pickup Zones:** Where boxes of various colors (Red, Blue, Green) are spawned.
* **Drop Zones:** Designated areas where robots must deliver boxes based on their color.
* **The Fleet:**
* `Glyco`: Specialized holonomic drive robot.
* `Crystal`: Specialized holonomic drive robot.
* `Frostbite`: Specialized holonomic drive robot.

## Playlist
[![Watch the video](https://github.com/nithishreddy1101/EYRC_Holonomic_Robots_Delivery_System/blob/main/bots.png)](https://www.youtube.com/playlist?list=PLXR0p4xqMjKSUtnwPgtP43irasatlMKYk)


## 🚀 Key Features

* **Holonomic Maneuverability:** All three robots utilize omnidirectional movement, allowing for precise positioning in tight warehouse aisles.
* **Dynamic Task Allocation:** An integrated scheduler assigns the nearest available robot to a newly detected box in the pickup zone.
* **Color-Based Logic:** Robots use Aruco markers  to identify box colors based on marker IDs numbers and navigate to the corresponding drop-off coordinate.
* **Collision Avoidance:** Multi-robot coordination to ensure `Glyco`, `Crystal`, and `Frostbite` do not interfere with each other's paths.

## 🛠️ Installation & Setup

### Prerequisites

* **OS:** Ubuntu 22.04 (Jammy Jellyfish)
* **ROS2:** Humble Hawksbill
* **Simulator:** Gazebo (Ignition)
* **Navigation:** ArUco marker tracking for precise docking at pickup zones.


## 🤖 Robot Specifications

| Robot Name | Drive Type | Primary Role |
| --- | --- | --- |
| **Glyco** | Holonomic/Omni | Heavy Lifting / Sorting |
| **Crystal** | Holonomic/Omni | High-Speed Transport |
| **Frostbite** | Holonomic/Omni | Precision Placement |
---