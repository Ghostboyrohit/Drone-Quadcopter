# Arduino Drone Quadcopter
#### Author: Bocaletto Luca
A fully featured 4-rotor drone with attitude stabilization, altitude hold, RC control, telemetry and safety features—all running on an Arduino Mega 2560.  

---

## 1. System Overview  
1. **Flight Controller**: Arduino Mega 2560  
2. **Sensors**:  
   - MPU-6050 (6-axis IMU via I²C/DMP) → roll, pitch, yaw rates & angles  
   - BMP280 (barometric pressure via I²C) → altitude  
   - Optional HMC5883L magnetometer → heading  
3. **Actuators**:  
   - 4 × ESC-driven brushless motors (CW/CCW propellers)  
4. **RC Receiver**: PWM input channels (roll, pitch, yaw, throttle)  
5. **Telemetry**: UART → XBee or Bluetooth MOD  
6. **Power**:  
   - LiPo 3S (11.1 V)  
   - 5 V BEC to Arduino & sensors  

---

## 2. Bill of Materials

| Qty | Part                                   | Note                                 |
|-----|----------------------------------------|--------------------------------------|
| 1   | Arduino Mega 2560                      | Flight controller                    |
| 1   | MPU-6050 IMU                           | I²C, DMP firmware                    |
| 1   | BMP280 barometer                       | I²C altitude sensor                  |
| 1   | (Optional) HMC5883L magnetometer       | I²C heading                          |
| 4   | ESC (20 A)                             | with BEC                              |
| 4   | Brushless motors & CW/CCW props        | 800–1200 KV                         |
| 1   | RC receiver (4-ch PWM)                 | e.g. FrSky, Spektrum, FlySky         |
| 1   | LiPo battery (3S 2200–5000 mAh)        | high-C rating                        |
| 1   | Power distribution board               | 4-way ESC plugs                      |
| 1   | XBee or HC-05 Bluetooth                | telemetry link                       |
| —   | Wires, connectors, zip-ties, standoffs  |                                      |

---

## 3. Wiring Diagram

      Arduino Mega        MPU-6050           BMP280            RC Receiver
      ┌─────────────────┐ ┌───────┐         ┌───────┐         ┌─────────────────────┐
      │ 5 V    ──► VIN  │ │ VCC   │         │ VIN   │         │ Ch1 OUT ──► D44     │
      │ GND    ──► GND  │ │ GND   │         │ GND   │         │ Ch2 OUT ──► D45     │
      │ SDA    ──► 20   │ │ SDA   │◄────────│ SDA   │         │ Ch3 OUT ──► D46     │
      │ SCL    ──► 21   │ │ SCL   │◄────────│ SCL   │         │ Ch4 OUT ──► D47     │
      │ INT    ──► 2    │ └───────┘         └───────┘         └─────────────────────┘
      └─────────────────┘

      Arduino Mega        ESC Outputs         Motors (CW/CCW)
      ┌─────────────────┐ ┌─────────────────┐ ┌───────────────────────────┐
      │ D6      ──► ESC1│─►│ PWM Out 1 ──►  │ M1 CW   │ Front-Left        │
      │ D7      ──► ESC2├─►│ PWM Out 2 ──►  │ M2 CCW  │ Front-Right       │
      │ D8      ──► ESC3│─►│ PWM Out 3 ──►  │ M3 CW   │ Rear-Right        │
      │ D9      ──► ESC4│─►│ PWM Out 4 ──►  │ M4 CCW  │ Rear-Left         │
      └─────────────────┘ └─────────────────┘ └───────────────────────────┘

- **ESC PWM**: standard 400 Hz PWM (0–2000 µs pulse) on D6–D9.  
- **RC Inputs**: pulseIn on D44–D47 (roll, pitch, throttle, yaw).  
- **I²C Bus**: SDA on 20, SCL on 21, pull-ups already on Mega.  

---

## 4. Flight Control Algorithm

1. **Initialization**  
   - Wake up MPU-6050 DMP for angle estimation  
   - Calibrate zero-offsets for gyro & barometer  

2. **RC Channel Mapping**  
   - Read raw PWM from receiver (1000–2000 µs)  
   - Map to desired setpoints:  
     - RollSet = (rollPWM − 1500) / 500 → ±1.0  
     - PitchSet = …  
     - YawSet = (yawPWM − 1500) / 500 → ±1.0  
     - Throttle = (thPWM − 1000) / 1000 → [0…1]  

3. **Sensor Feedback**  
   - Get `pitchAngle` & `rollAngle` from DMP  
   - Get `yawRate` from gyro (for heading control)  
   - Get `altitude` from BMP280 → altitude error  

4. **PID Controllers**  
   - `PID_roll` on (RollSet, rollAngle) → `corrRoll`  
   - `PID_pitch` on (PitchSet, pitchAngle) → `corrPitch`  
   - `PID_yaw` on (YawSet, gyroYawRate) → `corrYaw`  
   - `PID_alt` on (ThrottleSet, altitude) → `corrAlt`  

5. **Motor Mixing**  

       Motor mixing equations
       M1 = baseThr + corrPitch + corrRoll - corrYaw;
       M2 = baseThr + corrPitch - corrRoll + corrYaw;
       M3 = baseThr - corrPitch - corrRoll - corrYaw;
       M4 = baseThr - corrPitch + corrRoll + corrYaw;


6. **Safety**  
- Cut motors if tilt > 60° or RC signal lost  
- Failsafe throttle drop  

---

## 5. Software Setup

1. **Arduino IDE Libraries**  
- `Wire` (built-in)  
- `MPU6050` & `I2Cdev` (Jeff Rowberg)  
- `Adafruit_BMP280` (Adafruit)  
- `PID_v1` (Brett Beauregard)  
2. Create folder `QuadPro/` with:  
- `README.md` (this file)  
- `QuadPro.ino` (sketch below)  
3. Open `QuadPro.ino` in IDE, select **Arduino Mega 2560**, set COM port  

---

## 6. Take-Off & Tuning

1. **Idle Setup**  
- Propellers off; power on and let it sit to zero sensors  
- Check RC failsafe & arm sequence (e.g. throttle low + yaw right)  
2. **PID Tuning** (one axis at a time):  
- Increase `Kp` until small oscillations on axis  
- Add `Ki` to correct drift  
- Add `Kd` to damp overshoot  
3. **Hover Test** at low altitude (1 m indoors)  
4. **Altitude Hold** tuning with barometer  
5. **Flight Tests**: gentle  
6. **Failsafe Tests**: signal loss, tilt shutdown  

Refine gains and mixes incrementally. Enjoy your professional drone!  
