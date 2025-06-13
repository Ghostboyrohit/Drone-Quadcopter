// File: QuadPro.ino
// Professional 4-rotor flight controller on Arduino Mega 2560
// Author: Bocaletto Luca
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <PID_v1.h>

// ===== Pin Definitions =====
// ESC outputs
const uint8_t ESC1_PIN = 6;
const uint8_t ESC2_PIN = 7;
const uint8_t ESC3_PIN = 8;
const uint8_t ESC4_PIN = 9;
// RC inputs (pulseIn)
const uint8_t ROLL_CH_PIN     = 44;
const uint8_t PITCH_CH_PIN    = 45;
const uint8_t THROTTLE_CH_PIN = 46;
const uint8_t YAW_CH_PIN      = 47;

// ===== Objects =====
MPU6050 mpu;
Adafruit_BMP280 bmp;
double rollAngle, pitchAngle, yawRate;
double altitude, altitudeSet;

// PID setpoints & outputs
double rollSet, rollOut;
double pitchSet, pitchOut;
double yawSet, yawOut;
double thrSet, thrOut;  // thrOut is altitude correction

// PID tuning parameters (tweak in flight)
double Kp_r=4.0, Ki_r=0.02, Kd_r=2.0;
double Kp_p=4.0, Ki_p=0.02, Kd_p=2.0;
double Kp_y=3.0, Ki_y=0.01, Kd_y=1.0;
double Kp_a=2.0, Ki_a=0.01, Kd_a=1.5;

// PID objects
PID pidRoll(&rollAngle, &rollOut, &rollSet, Kp_r, Ki_r, Kd_r, DIRECT);
PID pidPitch(&pitchAngle, &pitchOut, &pitchSet, Kp_p, Ki_p, Kd_p, DIRECT);
PID pidYaw(&yawRate, &yawOut, &yawSet, Kp_y, Ki_y, Kd_y, DIRECT);
PID pidAlt(&altitude, &thrOut, &altitudeSet, Kp_a, Ki_a, Kd_a, DIRECT);

// ===== Utilities =====
// Read RC PWM channel (1000–2000 µs)
uint16_t readRC(uint8_t pin) {
  uint32_t t = pulseIn(pin, HIGH, 30000);
  return constrain(t, 1000, 2000);
}

// Map PWM to −1…1 or 0…1
double mapRC(double val, double inMin, double inMax, double outMin, double outMax) {
  return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Write ESC: expects 1000–2000 µs pulse
void writeESC(uint8_t pin, uint16_t pulse) {
  // use Arduino’s Servo library if preferred; here we bit-bang
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(pin, LOW);
}

// ===== Setup =====
void setup() {
  // Pins
  pinMode(ESC1_PIN, OUTPUT);
  pinMode(ESC2_PIN, OUTPUT);
  pinMode(ESC3_PIN, OUTPUT);
  pinMode(ESC4_PIN, OUTPUT);
  
  pinMode(ROLL_CH_PIN, INPUT);
  pinMode(PITCH_CH_PIN, INPUT);
  pinMode(THROTTLE_CH_PIN, INPUT);
  pinMode(YAW_CH_PIN, INPUT);

  Serial.begin(115200);
  Wire.begin();

  // Init MPU-6050
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  // Init BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 failed");
    while (1);
  }

  // Initialize PIDs
  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidYaw.SetMode(AUTOMATIC);
  pidAlt.SetMode(AUTOMATIC);
  pidRoll.SetOutputLimits(-200, 200);
  pidPitch.SetOutputLimits(-200, 200);
  pidYaw.SetOutputLimits(-200, 200);
  pidAlt.SetOutputLimits(-200, 200);

  // Zero references
  altitudeSet = 0;
  while (!mpu.dmpPacketAvailable()) {}
  sensors_event_t gyroEvent, accelEvent;
}

// ===== Main Loop =====
void loop() {
  // 1) Read RC
  double rollPWM     = readRC(ROLL_CH_PIN);
  double pitchPWM    = readRC(PITCH_CH_PIN);
  double thrPWM      = readRC(THROTTLE_CH_PIN);
  double yawPWM      = readRC(YAW_CH_PIN);

  rollSet  = mapRC(rollPWM, 1000, 2000, -1.0, 1.0);
  pitchSet = mapRC(pitchPWM,1000,2000,-1.0,1.0);
  yawSet   = mapRC(yawPWM,  1000,2000,-1.0,1.0);
  thrSet   = mapRC(thrPWM,  1000,2000, 0.0,1.0);

  // 2) Sensor feedback
  if (mpu.dmpGetCurrentFIFOPacket()) {
    mpu.dmpGetQuaternion(nullptr, nullptr);
    mpu.dmpGetYawPitchRoll(nullptr, &pitchAngle, &rollAngle);
    mpu.getRotation(&yawRate, nullptr, nullptr);
    yawRate   /= 131.0;  // convert to deg/s
  }
  altitude = bmp.readAltitude(1013.25);   // sea-level pressure

  // 3) Altitude setpoint ramp
  if (thrSet > 0.1) {
    altitudeSet += (thrSet - 0.1) * 0.02;  // slow climb
  }

  // 4) Compute PIDs
  pidRoll.Compute();
  pidPitch.Compute();
  pidYaw.Compute();
  pidAlt.Compute();

  // 5) Motor mixing (microseconds)
  int m1 = 1500 + thrPWM * 0.5 + rollOut + pitchOut - yawOut;
  int m2 = 1500 + thrPWM * 0.5 - rollOut + pitchOut + yawOut;
  int m3 = 1500 + thrPWM * 0.5 - rollOut - pitchOut - yawOut;
  int m4 = 1500 + thrPWM * 0.5 + rollOut - pitchOut + yawOut;
  m1 = constrain(m1, 1100, 1900);
  m2 = constrain(m2, 1100, 1900);
  m3 = constrain(m3, 1100, 1900);
  m4 = constrain(m4, 1100, 1900);

  // 6) Output to ESCs
  writeESC(ESC1_PIN, m1);
  writeESC(ESC2_PIN, m2);
  writeESC(ESC3_PIN, m3);
  writeESC(ESC4_PIN, m4);

  // 7) Safety: if extreme tilt, cut motors
  if (abs(rollAngle) > 60 || abs(pitchAngle) > 60) {
    writeESC(ESC1_PIN, 1000);
    writeESC(ESC2_PIN, 1000);
    writeESC(ESC3_PIN, 1000);
    writeESC(ESC4_PIN, 1000);
    while (1) {}
  }
}
