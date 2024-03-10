# Robot Car with LiDAR

Robot Car with LiDAR is a project to build a robot car with a LiDAR sensor to navigate the environment and avoid obstacles. The car uses a tank track with 4 DC motors. It can be controlled by a remote control or can be driven autonomously.

There are 3 operational modes: `Manual`, `Autonomous`, and `Semi-Auto`. 
- `Manual` mode is controlled by a remote control.
- `Autonomous` mode is controlled by the LiDAR sensor.
- `Semi-Auto` mode is a combination of the `Manual` and `Autonomous` modes, it can avoid obstables even when it is controlled by the RC.

### RP LiDAR Pin out:
**Motor**
- `Purple` wire is the LiDAR `PWM wire` (Pin 8)
- `Yellow` wire is the LiDAR `Motor GND` wire
- `Red` wire is the LiDAR `Motor +5v` wire

The LiDAR motor is running at 9v, so the `PWM wire` is connected to the `Arduino Mega` `Pin 8` to control the motor speed.

**LiDAR**
- `White` wire is the LiDAR `Ground` wire
- `Black` wire is the LiDAR `5v` wire
- `Orange` wire is the LiDAR `RX` wire
- `Green` wire is the LiDAR `TX` wire

### Components:
- Arduino Mega 2560 REV3 (https://amzn.to/3wObNdW)
- Slamtec RPLIDAR A1M8 (https://amzn.to/3VaSoy3)
- 7.4V 1200mAh Lipo Battery (https://amzn.to/3Pdja4Z)
- L298N Motor Driver Controller Board (https://amzn.to/3TxBqc0)
- FLYSKY FS-i6X 10CH 2.4GHz RC Transmitter (https://amzn.to/3wP557x)
