/*
  nano_motor_control.ino

  Low-level controller for the WRO Future Engineers robot.
  Runs on an Arduino Nano, receiving drive commands from the Raspberry
  Pi 5 over hardware Serial (D0/D1) and driving:
    - L298N motor driver  (ENA on D5, IN1 on D6, IN2 on D7)
    - Steering servo      (signal on D9)
  while reading:
    - 1x VL53L0X time-of-flight sensor (front-facing), I2C, default address
      (0x29) -- no XSHUT, no multiplexer needed with just one sensor on
      the bus. When you add the TCA9548A mux later to bring the other
      three sensors online, this is the piece that changes.
    - MPU-9250/6500 IMU, accelerometer + gyro (I2C)
      Talked to directly via raw register reads rather than a library --
      the accel/gyro register map is identical to the MPU6050's, so this
      one driver works for either chip without pulling in a dependency
      that only recognizes one of them.

  --------------------------------------------------------------------
  Serial protocol, Pi5 -> Nano, one line per command:
      M<speed>,<steer>\n
        speed : -255..255  (negative = reverse)
        steer : -30..30    (degrees from center, positive = right)
      e.g. "M150,-10\n"

  Telemetry, Nano -> Pi5, sent every TELEMETRY_INTERVAL_MS:
      T,<tof_front_mm>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n
      (ToF value is -1 if the sensor isn't initialized or out of range.
       accel in g, gyro in deg/s.)
  --------------------------------------------------------------------

  Dependencies (install via Arduino Library Manager):
    - Adafruit VL53L0X
    (no IMU library needed -- see note above)

  IMPORTANT: upload this sketch over USB with the Pi5 link DISCONNECTED
  from D0/D1 -- those pins are shared with the USB programming interface.
  Reconnect the Pi5's TX/RX (through the voltage divider on the Nano TX
  line) only after the sketch is flashed.
*/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// ---- Pin assignments ----
const uint8_t PIN_ENA = 3;   // PWM speed
const uint8_t PIN_IN1 = 2;
const uint8_t PIN_IN2 = 4;
const uint8_t PIN_SERVO = 9;
// ToF and IMU both sit on the shared I2C bus (A4/A5) -- no dedicated pins needed.

// ---- MPU-9250/6500 register-level access (also works for MPU6050 --
//      the accel/gyro registers are identical across all three chips) ----
const uint8_t IMU_ADDR = 0x68;          // AD0 pin low; use 0x69 if tied high
const uint8_t REG_WHO_AM_I = 0x75;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

const float ACCEL_SCALE = 8.0f / 32768.0f;    // g per LSB, configured for +/-8g
const float GYRO_SCALE = 500.0f / 32768.0f;   // deg/s per LSB, configured for +/-500 dps

const int SERVO_CENTER_DEG = 83;  // calibrated on the real 180-degree servo via
                                   // nano_test_drive.ino's live trim tool -- not
                                   // a generic 90, this is specific to this servo
                                   // horn/linkage assembly
const unsigned long TELEMETRY_INTERVAL_MS = 50;
const unsigned long COMMAND_TIMEOUT_MS = 300;  // stop motor if Pi5 goes quiet

Servo steeringServo;
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

bool tofReady = false;
bool imuReady = false;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

int currentSpeed = 0;
int currentSteer = 0;
unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;

String serialBuffer = "";

void setMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, speed);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, -speed);
  }
}

void setSteering(int deg) {
  deg = constrain(deg, -30, 30);
  steeringServo.write(SERVO_CENTER_DEG + deg);
}

void initToFSensor() {
  tofReady = tof.begin();  // default address 0x29 -- fine with just one sensor on the bus
  if (!tofReady) {
    Serial.println("ERR: ToF init failed");
  }
}

int readToFmm() {
  if (!tofReady) return -1;
  VL53L0X_RangingMeasurementData_t m;
  tof.rangingTest(&m, false);
  return (m.RangeStatus != 4) ? m.RangeMilliMeter : -1;
}

bool imuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuReadRegisters(uint8_t startReg, uint8_t *buffer, uint8_t count) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  Wire.requestFrom(IMU_ADDR, count);
  for (uint8_t i = 0; i < count && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool imuInit() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) {
    return false;  // nothing answered at this address at all
  }
  Wire.requestFrom(IMU_ADDR, (uint8_t)1);
  if (!Wire.available()) return false;
  uint8_t whoAmI = Wire.read();

  // Expect 0x71/0x73 (MPU9250), 0x70 (MPU6500), or 0x68 (MPU6050) --
  // print it either way so you can confirm you're talking to the chip
  // you think you are.
  Serial.print("IMU WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  bool ok = true;
  ok &= imuWriteRegister(REG_PWR_MGMT_1, 0x00);   // wake up from sleep
  delay(10);
  ok &= imuWriteRegister(REG_GYRO_CONFIG, 0x08);  // +/-500 dps
  ok &= imuWriteRegister(REG_ACCEL_CONFIG, 0x10); // +/-8g
  return ok;
}

bool imuRead(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t raw[14];
  if (!imuReadRegisters(REG_ACCEL_XOUT_H, raw, 14)) return false;

  int16_t rawAx = (raw[0] << 8) | raw[1];
  int16_t rawAy = (raw[2] << 8) | raw[3];
  int16_t rawAz = (raw[4] << 8) | raw[5];
  // raw[6..7] is the onboard temperature sensor -- unused here
  int16_t rawGx = (raw[8] << 8) | raw[9];
  int16_t rawGy = (raw[10] << 8) | raw[11];
  int16_t rawGz = (raw[12] << 8) | raw[13];

  ax = rawAx * ACCEL_SCALE;
  ay = rawAy * ACCEL_SCALE;
  az = rawAz * ACCEL_SCALE;
  gx = rawGx * GYRO_SCALE - gyroBiasX;
  gy = rawGy * GYRO_SCALE - gyroBiasY;
  gz = rawGz * GYRO_SCALE - gyroBiasZ;
  return true;
}

void calibrateGyro(uint16_t samples = 200) {
  // Averages raw gyro output while assumed stationary (robot sitting in
  // the start zone at power-on, per the competition procedure) and stores
  // it as the zero-offset for imuRead() to subtract going forward. Without
  // this, small per-axis bias integrates into fake heading drift over a
  // multi-second run -- exactly what the Pi5's lap counter would misread.
  Serial.println("Calibrating gyro -- keep the robot still...");
  float sumX = 0, sumY = 0, sumZ = 0;
  uint16_t collected = 0;

  for (uint16_t i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    if (imuRead(ax, ay, az, gx, gy, gz)) {  // bias is still 0 at this point
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      collected++;
    }
    delay(3);
  }

  if (collected > 0) {
    gyroBiasX = sumX / collected;
    gyroBiasY = sumY / collected;
    gyroBiasZ = sumZ / collected;
  }

  Serial.print("Gyro bias (deg/s): X=");
  Serial.print(gyroBiasX, 3);
  Serial.print(" Y=");
  Serial.print(gyroBiasY, 3);
  Serial.print(" Z=");
  Serial.println(gyroBiasZ, 3);
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== nano_motor_control.ino -- Pi5 M<speed>,<steer> protocol ===");

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  setMotor(0);

  steeringServo.attach(PIN_SERVO);
  setSteering(0);

  Wire.begin();
  initToFSensor();

  imuReady = imuInit();
  if (!imuReady) {
    Serial.println("ERR: IMU init failed (check wiring / I2C address)");
  } else {
    calibrateGyro();
  }

  lastCommandMs = millis();
}

void handleCommand(const String &line) {
  // Expected format: M<speed>,<steer>
  if (line.length() < 2 || line[0] != 'M') return;

  int commaIndex = line.indexOf(',');
  if (commaIndex < 0) return;

  int speed = line.substring(1, commaIndex).toInt();
  int steer = line.substring(commaIndex + 1).toInt();

  currentSpeed = speed;
  currentSteer = steer;
  lastCommandMs = millis();

  setMotor(currentSpeed);
  setSteering(currentSteer);
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(serialBuffer);
      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
    }
  }
}

void sendTelemetry() {
  int tofMm = readToFmm();

  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (imuReady) {
    imuRead(ax, ay, az, gx, gy, gz);
  }

  Serial.print("T,");
  Serial.print(tofMm); Serial.print(",");
  Serial.print(ax, 3); Serial.print(",");
  Serial.print(ay, 3); Serial.print(",");
  Serial.print(az, 3); Serial.print(",");
  Serial.print(gx, 3); Serial.print(",");
  Serial.print(gy, 3); Serial.print(",");
  Serial.println(gz, 3);
}

void loop() {
  readSerial();

  // Safety: stop driving if the Pi5 link goes quiet (crash, disconnect,
  // loose wire) rather than continuing on the last command forever.
  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    setMotor(0);
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }
}
