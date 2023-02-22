# MIT_Robotic_Arm
Repository for the Making Intelligent Things Project's Documentation

<div>
   <video src="./demo_arm.mp4" width=480/>
</div>Update with demo video

## Summary

1. [Robotic Arm and Car Assembly](#1-arm-and-car-assembly)
   1. [3D Prints' Files](#i-3d-printed-files-resources)
   2. [Assembly Instructions](#ii-assembly-instructions)
2. [Arm Controlling Applications](#2-arms-controls-application)
   1. [Shopping Application](#i-shopping-application)
   2. [ESP32 CAM Code](#ii-esp32-cam-code)
   3. [Arduino Code](#iii-arduino-code)
   4. [Manual Arm Control Application](#iv-manual-arm-control-application)

## 1. Arm And Car Assembly

### i. 3D Printed Files Resources

You can find all the printable .stl files used in the project in this [folder](/printable_stls) in this 
repository.

### ii. Assembly Instructions

Detailed assembly instructions can be found [here](/assembly_instructions) in this repository.

## 2. Arm's Controls Application

### i. Shopping Application

A GitHub Repository for the Shopping Application's code and documentation can be found 
[here](https://github.com/Zephyr75/bobby_shopping).

### ii. ESP32 CAM Code
ESP32-Cam documentation:

A GitHub Repository for the ESP32CAM's Code and Documentation can be found
[here](https://github.com/Aco-Hub/ESP32-Cam-Shopping).

The ESP32-CAM has 3 main functions:

1. Communicate with the app
2. Scan QR code
3. Control the behavior of the arduino
#### 1. Communicate with the app:

We use UDP over my phone WI-FI to give the JSON shoppingList and confirmation
when an article is scanned. We used TCP to communicate qr code images because
it was easier.

#### 2. Scan QR code:

The esp32 cam gives a screenshot to the app and the app analyzes the screenshot
and gives the resulting payload to the esp32 cam. We use TCP to give the image 
to the app and UDP to receive the payload.

#### 3. Communication with the arduino:

We communicate with RX and TX software pins. The esp32 cam gives orders to
the arduino like forward and the arduino obey and confirm that the order has 
been received. The frames of the animation of the arm were too big so we decided 
to store them on the arduino and communicate the animation frame per frame.

### iii. Arduino Code

The documented Arduino Code can be found [here](/arduino_code/arduino_final_program.ino).
in this repository

### iv. Manual Arm Control Application

A GitHub Repository for the Arm Control Application and Documentation can be found
[here](https://github.com/WollfieGitHub/SerialArmCommunication).
