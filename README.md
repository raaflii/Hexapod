# Hexapod Robot

> [!WARNING]  
> This project is currently under development!

This project is focusing on hexapod movement control and implemets AI in the robot. The system operates within a Cartesian coordinate framework with the following axis directions:

- X positive → Right / left
- Y positive → Fordward / Backward
- Z positive → Upward / Downward

### Hardware

- Esp32 Dev Module
- Servo MG996R
- PCA9685
- Buck Converter 20A 300W
- MPU6050

### System

This hexapod using tripod gait, cubic bezier curve for trajectory planning, and MPU6050 for estimate pitch and roll error with Kalman Filter. Hexapod also has remote controller via website with Nipple.js (CDN).

### AI

> [!WARNING]  
> Under Project!

Using Neuro-Fuzzy for adaptive and responsive movement when moving on Irregular environment. (Bakal dilanjutin kalau ada waktu :sweat:)

Bismillah Sukses :grin:
