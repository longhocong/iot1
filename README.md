# Temperature and Light Intensity Measurement System
## Trash Detection and Arm Control Integrated with IoT Monitoring
### Powered by Adafruit IO Server and Mobile App

#### Project Description

This repository houses the code and documentation for a multifunctional system that integrates temperature and light intensity measurement, trash detection, and arm control through an IoT infrastructure. The project employs an STM32 microcontroller to collect data from various sensors and transmit it to the Adafruit IO server. Additionally, a basic Android application has been developed to facilitate remote control and monitoring of the system using the MQTT protocol for communication.

A computer serves as the central gateway, facilitating data exchange between the server, the STM32 microcontroller, and the computer's camera, which employs AI (leveraging Google Teachable Machine) for trash detection. The STM32 microcontroller is responsible for sensor measurements, LED control, and arm manipulation via three servo sg90 motors, following commands learned from the arm using a potentiometer. When trash is detected and sent from the computer to the microcontroller through UART, the arm swings into action to collect the trash.

Users have the flexibility to monitor and temporarily pause the system via the server and a dedicated mobile app. The system is equipped with robust error-handling mechanisms to address any issues related to data transmission and reception.

#### Features

- Precise measurement of temperature and light intensity using the STM32 microcontroller and advanced sensors.
- Smart trash detection system utilizing AI and computer vision via Google Teachable Machine.
- Control of a robotic arm equipped with three servo sg90 motors.
- Seamless data communication with the Adafruit IO server through the MQTT protocol.
- Mobile application for convenient system monitoring and control.
- Robust error-handling mechanisms ensuring smooth data transmission and reception.

#### Folder Structure

- **STM32_Code**: This folder contains the source code for the STM32 microcontroller.
- **Mobile_App**: Here, you can find the source code for the mobile application.
- **Computer_Gateway**: This folder includes the code for the computer acting as a central gateway.
- **Documentation**: You'll find additional project documentation and resources here.

For detailed setup instructions, code execution guidelines, and additional project details, please refer to the documentation folder.

We hope you find this project informative and inspiring!

