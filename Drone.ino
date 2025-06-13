// File: QuadPro.ino
// Professional 4-rotor flight controller on Arduino Mega 2560
// Author: Bocaletto Luca
// Features:
// - Attitude stabilization with MPU6050 DMP (roll, pitch, yaw)
// - Altitude hold with BMP280 barometer
// - RC input reading with failsafe & arming protocol
// - ESC control via Servo library (1000–2000 µs pulses)
// - Battery voltage monitoring with low-voltage cutoff
// - Telemetry output on Serial1 in JSON format
// - Modular PID controllers for roll, pitch, yaw & altitude

#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Adafruit_BMP280.h>
#include <PID_v1.h>

//=== Hardware Configuration ===
// ESC PWM outputs
const uint8_t ESC_PINS[4] = {6, 7, 8, 9};
Servo esc[4];

// RC input channels (roll, pitch, throttle, yaw)
const uint8_t RC_PINS[4] = {44, 45, 46, 47};

// Battery voltage monitor
const uint8_t BATT_PIN        = A1;
const float   BATT_DIV_RATIO  = 11.0;  // voltage divider ratio for 3S LiPo

// I2C addresses
#define MPU_ADDR 0x68
#define BMP_ADDR 0x76

//=== Flight Control Parameters ===
bool armed = false;
unsigned long lastRC[4] = {0,0,0,0};
const uint16_t RC_TIMEOUT     = 500;    // ms before RC channel considered lost

// Setpoints and feedback variables
double rollSet, pitchSet, yawSet, altSet;
double rollAngle, pitchAngle, yawRate, altitude;

// PID outputs
double rollOut, pitchOut, yawOut, altOut;

// PID tuning constants
double Kp_r=4.0, Ki_r=0.02, Kd_r=2.0;
double Kp_p=4.0, Ki_p=0.02, Kd_p=2.0;
double Kp_y=3.0, Ki_y=0.01, Kd_y=1.0;
double Kp_a=2.0, Ki_a=0.01, Kd_a=1.5;

// PID controllers
PID pidRoll(&rollAngle,  &rollOut, &rollSet, Kp_r, Ki_r, Kd_r, DIRECT);
PID pidPitch(&pitchAngle,&pitchOut,&pitchSet,Kp_p, Ki_p, Kd_p, DIRECT);
PID pidYaw(&yawRate,    &yawOut,  &yawSet,  Kp_y, Ki_y, Kd_y, DIRECT);
PID pidAlt(&altitude,   &altOut,  &altSet,  Kp_a, Ki_a, Kd_a, DIRECT);

// Sensor objects
MPU6050        mpu;
Adafruit_BMP280 bmp;

// DMP interrupt flag
volatile bool dmpReady = false;
void dmpISR() { dmpReady = true; }

//=== Helper Functions ===
// Read RC pulse width in µs (1000–2000), zero if no pulse
uint16_t readPulse(uint8_t pin) {
  uint32_t d = pulseIn(pin, HIGH, 30000);
  return d ? constrain(d, 1000, 2000) : 0;
}

// Map RC pulse to [-1,1] or [0,1]
double mapRC(double v, double inMin, double inMax, double outMin, double outMax) {
  return (v - inMin)*(outMax-outMin)/(inMax-inMin) + outMin;
}

// Read battery voltage via divider
float readBattery() {
  int raw = analogRead(BATT_PIN);
  float v = (raw/1023.0)*5.0*BATT_DIV_RATIO;
  return v;
}

// Send telemetry JSON on Serial1
void sendTelemetry() {
  Serial1.print("{\"r\":"); Serial1.print(rollAngle,1);
  Serial1.print(",\"p\":");        Serial1.print(pitchAngle,1);
  Serial1.print(",\"y\":");        Serial1.print(yawRate,1);
  Serial1.print(",\"h\":");        Serial1.print(altitude,1);
  Serial1.print(",\"b\":");        Serial1.print(readBattery(),2);
  Serial1.println("}");
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(57600);           // telemetry port

  // Attach ESCs and initialize to min throttle
  for (uint8_t i=0; i<4; i++) {
    esc[i].attach(ESC_PINS[i], 1000, 2000);
    esc[i].writeMicroseconds(1000);
  }

  // Configure RC inputs
  for (uint8_t i=0; i<4; i++) pinMode(RC_PINS[i], INPUT);

  // Battery pin
  pinMode(BATT_PIN, INPUT);

  // I2C & sensors
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(2), dmpISR, RISING);
  if (!bmp.begin(BMP_ADDR)) {
    Serial.println("BMP280 init failed"); while (1);
  }

  // PID setup
  pidRoll.SetMode(AUTOMATIC);   pidRoll.SetOutputLimits(-200,200);
  pidPitch.SetMode(AUTOMATIC);  pidPitch.SetOutputLimits(-200,200);
  pidYaw.SetMode(AUTOMATIC);    pidYaw.SetOutputLimits(-200,200);
  pidAlt.SetMode(AUTOMATIC);    pidAlt.SetOutputLimits(-200,200);
}

void loop() {
  // 1) Read & validate RC channels
  double rc[4];
  for (uint8_t i=0; i<4; i++) {
    rc[i] = readPulse(RC_PINS[i]);
    if (rc[i]>0) lastRC[i] = millis();
    else if (millis()-lastRC[i] > RC_TIMEOUT) {
      // throttle lost → disarm, others center
      rc[i] = (i==2 ? 1000 : 1500);
      armed = false;
    }
  }

  // 2) Arming/disarming gestures
  if (!armed && rc[3]<1100 && rc[2]<1100 && rc[0]>1400 && rc[1]>1400) {
    armed = true; Serial.println("ARMED");
  }
  if (armed && rc[3]>1900) {
    armed = false; Serial.println("DISARMED");
  }

  // 3) Map RC to setpoints
  rollSet  = mapRC(rc[0],1000,2000,-1,1);
  pitchSet = mapRC(rc[1],1000,2000,-1,1);
  altSet   = mapRC(rc[2],1000,2000, 0,1);
  yawSet   = mapRC(rc[3],1000,2000,-1,1);

  // 4) Get sensor feedback
  if (dmpReady) {
    dmpReady = false;
    Quaternion q; VectorFloat g;
    mpu.dmpGetCurrentFIFOPacket();
    mpu.dmpGetQuaternion(&q, NULL);
    mpu.dmpGetGravity(&g, &q);
    mpu.dmpGetYawPitchRoll(&yawRate, &pitchAngle, &rollAngle, &q);
    yawRate *= 180.0/3.14159;  // rad/s → deg/s
  }
  altitude = bmp.readAltitude(1013.25);

  // 5) Compute PID outputs
  pidRoll.Compute(); pidPitch.Compute();
  pidYaw.Compute();  pidAlt.Compute();

  // 6) Motor mixing
  double baseThr = 1000 + altSet*1000;
  int mix[4];
  mix[0] = constrain(baseThr + pitchOut + rollOut - yawOut, 1000, 2000);
  mix[1] = constrain(baseThr + pitchOut - rollOut + yawOut, 1000, 2000);
  mix[2] = constrain(baseThr - pitchOut - rollOut - yawOut, 1000, 2000);
  mix[3] = constrain(baseThr - pitchOut + rollOut + yawOut, 1000, 2000);

  // 7) Send signals to ESCs
  for (uint8_t i=0; i<4; i++) {
    esc[i].writeMicroseconds(armed ? mix[i] : 1000);
  }

  // 8) Low-battery failsafe
  if (readBattery() < 3.3*3) {
    armed = false;
    Serial.println("LOW BATT! DISARMED");
  }

  // 9) Telemetry
  static unsigned long lastTel = 0;
  if (millis() - lastTel > 200) {
    sendTelemetry();
    lastTel = millis();
  }
}
