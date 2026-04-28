# Autonomous Ultrasound-Guided Robotic Thoracentesis with Force-Controlled Scanning and Real-Time Attention U-Net Segmentation

This repository contains the implementation developed for an MSc Robotics dissertation project on **autonomous ultrasound-guided robotic thoracentesis**. The system improves an existing robotic thoracentesis platform by integrating:

- a redesigned lightweight end-effector,
- force-controlled ultrasound scanning using a Franka Emika Panda robot,
- real-time ultrasound image segmentation using U-Net-based models,
- Jetson AGX Orin-based perception and motor control,
- ROS-based communication between the robot-control PC and Jetson,
- automated needle insertion and peristaltic pump aspiration.

The system was validated on a **Kyoto Kagaku thoracentesis phantom** using a **Sonon 300L ultrasound scanner** and a **Franka Emika Panda robotic arm**.

---

## Project Overview

Thoracentesis is a minimally invasive medical procedure used to remove excess pleural fluid from the pleural space. Manual thoracentesis depends heavily on clinician experience, stable ultrasound probe handling, accurate interpretation of ultrasound images, and safe needle placement.

This project develops a prototype autonomous robotic workflow that can:

1. Move the robot to a predefined home position.
2. Move the ultrasound scanner to the scan start position.
3. Trigger real-time ultrasound segmentation on the Jetson.
4. Scan the curved thoracentesis phantom using force-controlled surface following.
5. Detect pleural effusion and bone regions from live ultrasound images.
6. Identify a safe needle insertion zone.
7. Pause the robot when a suitable insertion zone is detected.
8. Trigger the needle and pump sequence on the Jetson.
9. Extract liquid using a 3D-printed peristaltic pump.
10. Retract the needle.
11. Return the robot to the home position.

---

## Main Contributions

This project implements the following major improvements over the baseline robotic thoracentesis system:

### 1. Lightweight End-Effector Redesign

A new end-effector was designed in Fusion 360 and 3D printed using PLA for prototype testing. The design aimed to:

- reduce end-effector bulk,
- reduce distal payload,
- secure the Sonon 300L ultrasound scanner,
- provide a guided needle path,
- include space for wiring and tubing,
- minimise the number of motors mounted on the robot tip,
- reduce sharp edges,
- improve practical handling during contact-based ultrasound scanning.

The redesigned end-effector reduced the carried load from approximately **2 kg** in the previous setup to approximately **650 g**.

### 2. Servo-Driven Needle Pusher

The needle insertion mechanism uses a servo-driven pusher and gear arrangement. The servo moves the needle down during aspiration and retracts it after the pump sequence is complete.

### 3. 3D-Printed Peristaltic Pump

The previous syringe-based aspiration system was replaced with a 3D-printed peristaltic pump. The pump is placed near the robot base instead of on the end-effector to reduce distal payload.

The peristaltic pump is driven by a 12 V DC motor and connected to the needle through silicone tubing.

### 4. Real-Time Ultrasound Segmentation

Three segmentation models were implemented and compared:

- U-Net
- Attention U-Net
- U-Net++

The models segment ultrasound images into three classes:

- background,
- pleural effusion,
- bone.

The best-performing model, **Attention U-Net**, was deployed for real-time inference on the Jetson AGX Orin.

### 5. Force-Controlled Robotic Ultrasound Scanning

The Franka Emika Panda robot scans the curved thoracentesis phantom using a custom force scan controller. The scanning method combines:

- Cartesian impedance control,
- filtered force feedback,
- PI force regulation,
- bounded multi-pass scanning,
- parabolic surface approximation,
- probe rocking based on local surface slope,
- automatic return-to-home behaviour.

### 6. PC-Jetson Distributed Architecture

The original goal was to run the complete system from the Jetson. However, reliable Franka control requires stable low-latency communication and a real-time Linux setup. Therefore, the final system uses a distributed architecture:

- **PC**: robot control, MoveIt, Pilz, libfranka, force scan controller.
- **Jetson AGX Orin**: real-time ultrasound segmentation, safe-zone detection, servo control, pump control.

This improves reliability and prevents the Jetson from being overloaded with both GPU inference and real-time robot control.

---

## Hardware Used

### Robot and Ultrasound

- Franka Emika Panda robot
- Sonon 300L ultrasound scanner
- Kyoto Kagaku thoracentesis phantom
- Sonon tablet application
- Ethernet network connection between PC and Jetson

### Computing Hardware

- Ubuntu 20.04 PC
- Jetson AGX Orin
- Tablet running Sonon ultrasound application

### Actuation Hardware

- MG995 servo motor for needle pusher
- 12 V DC motor for peristaltic pump
- Grove I2C motor driver
- PCA9685 servo driver
- L298N motor driver / I2C motor driver setup depending on configuration
- Silicone tubing
- 22-gauge needle
- 3D-printed end-effector
- 3D-printed peristaltic pump

---

## Software Stack

### PC Side

The PC is responsible for Franka robot control.

- Ubuntu 20.04
- ROS Noetic
- MoveIt
- Pilz industrial motion planner
- libfranka
- franka_ros
- C++
- Cartesian impedance control
- Force PI control
- ROS services and parameters for controller switching and Jetson communication

### Jetson Side

The Jetson is responsible for perception and actuation.

- Ubuntu 20.04
- ROS Noetic
- Python 3
- PyTorch
- OpenCV
- NumPy
- mss
- scrcpy
- smbus2
- Adafruit PCA9685 library
- I2C motor-control libraries

---

## Repository Structure

A typical structure for this repository is shown below.

```text
.
├── README.md
├── robot_control/
│   ├── move_to_scan_start.cpp
│   ├── force_scan_controller.cpp
│   ├── controller_config.yaml
│   └── launch/
│       └── scan_sequence.launch
│
├── jetson_realtime_segmentation/
│   ├── realtime_scrcpy_attention_unet.py
│   ├── realtime_scrcpy_unet.py
│   ├── realtime_scrcpy_unetpp.py
│   ├── attention_unet_model.py
│   ├── unet_model.py
│   ├── unetpp_model.py
│   └── utils/
│       ├── preprocessing.py
│       ├── postprocessing.py
│       └── safe_zone_detection.py
│
├── model_training/
│   ├── train_unet.py
│   ├── train_attention_unet.py
│   ├── train_unetpp.py
│   ├── compare_models.py
│   └── metrics.py
│
├── motor_control/
│   ├── needle_pump_sequence.py
│   ├── servo_test.py
│   ├── dc_motor_test.py
│   └── i2c_scan.py
│
├── cad/
│   ├── end_effector/
│   └── peristaltic_pump/
│
├── results/
│   ├── segmentation_metrics/
│   ├── detected_frames/
│   ├── extraction_trials/
│   └── comparison_plots/
│
├── dataset/
│   ├── images/
│   ├── masks/
│   ├── train/
│   ├── val/
│   └── test/
│
└── docs/
    ├── dissertation_report.pdf
    ├── system_architecture.png
    └── demo_video_link.txt

## System Architecture
                 ┌────────────────────────────┐
                 │       Ubuntu PC             │
                 │  ROS Noetic + MoveIt + Pilz │
                 │  Franka robot control       │
                 └──────────────┬─────────────┘
                                │ ROS over Ethernet
                                │
                 ┌──────────────▼─────────────┐
                 │      Jetson AGX Orin        │
                 │  Real-time segmentation     │
                 │  Safe-zone detection        │
                 │  Servo and pump control     │
                 └──────────────┬─────────────┘
                                │
          ┌─────────────────────┼─────────────────────┐
          │                     │                     │
 ┌────────▼────────┐   ┌────────▼────────┐   ┌────────▼────────┐
 │ Sonon Tablet    │   │ Needle Servo    │   │ Peristaltic Pump│
 │ via scrcpy      │   │ via PCA9685     │   │ via Motor Driver│
 └─────────────────┘   └─────────────────┘   └─────────────────┘
