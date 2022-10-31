# Ardunio FOC Force Feedback Control

This is a proof of concept project to determine if a force feedback joystick can be created with stepper motors, SimpleFOC and X-Plane. This library uses XPlaneConnect from [NASA](https://github.com/nasa/XPlaneConnect). 

### Hardware:
- ESP8266 NodeMCU
- Stepper Motor
- L298N Driver
- AS5600 Encoder
  - Do not use magnetic encoders, use mechanical encoders. This will be changes in future