# Car Robot with LiDAR

### Components:
- Arduino Mega 2560 REV3 (https://amzn.to/3wObNdW)
- Slamtec RPLIDAR A1M8 (https://amzn.to/3VaSoy3)
- 7.4V 1200mAh Lipo Battery (https://amzn.to/3Pdja4Z)
- L298N Motor Driver Controller Board (https://amzn.to/3TxBqc0)
- FLYSKY FS-i6X 10CH 2.4GHz RC Transmitter (https://amzn.to/3wP557x)

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
