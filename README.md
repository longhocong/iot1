PROJECT TITLE: TEMPERATURE AND LIGHT INTENSITY MEASUREMENT SYSTEM, TRASH DETECTION, AND ARM CONTROL INTEGRATED WITH IOT MONITORING VIA ADAFRUIT IO SERVER AND MOBILE APP

Description:
This project aims to develop a system that measures temperature and light intensity, detects trash, and controls an arm using an integrated IoT infrastructure. The system utilizes an STM32 microcontroller to collect data from sensors and send it to the Adafruit IO server. A basic Android application is developed to receive and control the system, using the MQTT protocol for communication. A computer acts as a gateway to exchange data between the server, STM32 microcontroller, and the computer's camera, which is used for trash detection through AI (using Google Teachable Machine). The STM32 microcontroller is responsible for sensor measurements, LED control, and controlling the arm (equipped with three servo sg90 motors) based on commands learned from the arm using a potentiometer. When trash is detected and sent from the computer to the microcontroller via UART, the arm will pick up the trash. Users can monitor and temporarily pause the system through the server and the mobile app. The system includes error handling mechanisms to address issues related to data transmission and reception.

Readme:

Project Description
This repository contains the code and documentation for the Temperature and Light Intensity Measurement System, Trash Detection, and Arm Control integrated with IoT monitoring via the Adafruit IO server and a mobile app.

Features

Measurement of temperature and light intensity using STM32 microcontroller and sensors.
Trash detection using AI (Google Teachable Machine) with a computer's camera.
Control of an arm with three servo sg90 motors.
Data communication with the Adafruit IO server using MQTT protocol.
Mobile app for system monitoring and control.
Error handling mechanisms for data transmission and reception.
Folder Structure

STM32_Code: Contains the code for the STM32 microcontroller.
Mobile_App: Contains the code for the mobile application.
Computer_Gateway: Contains the code for the computer acting as a gateway.
Documentation: Contains additional project documentation and resources.
