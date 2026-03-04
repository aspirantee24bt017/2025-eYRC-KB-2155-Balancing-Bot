/*
 * =============================================================================
 *  SELF-BALANCING TWO-WHEELED ROBOT - Cascaded PID Control
 * =============================================================================
 *
 *  Hardware:
 *    - Arduino Nano (ATmega328P, 16 MHz)
 *    - MPU6050 6-axis IMU (I2C)
 *    - 2x N20 300RPM DC geared motors with quadrature encoders
 *    - L298N motor driver IC (e-Yantra botController v1.1 PCB)
 *    - 7.4V 2S LiPo battery
 *    - Buzzer on A1
 *
 *  Control Architecture (cascaded, three-level):
 *    1. Inner Loop  (100 Hz) - Angle PID: keeps robot upright using IMU
 *    2. Outer Loop  (20 Hz)  - Velocity PI: controls forward/backward speed
 *                               using encoder deltas (NOT accumulated position)
 *    3. Yaw Control (20 Hz)  - PD: differential drive for turning
 *
 *  Why encoder drift is avoided:
 *    - Balance depends ONLY on IMU angle/rate (inner loop)
 *    - Velocity uses SHORT-TERM encoder tick deltas each 50ms cycle
 *    - No long-term position integration => no drift accumulation
 *
 *  Bluetooth (HC-05 on SoftwareSerial pins 10/11, Dabble app):
 *    - D-pad: Forward/Back/Left/Right movement
 *    - Triangle/Cross: increase/decrease Kp_angle
 *    - Circle/Square: increase/decrease baseTargetAngle
 *    - USB Serial (pins 0/1) free for debug telemetry
 *
 * =============================================================================
 */

// ─────────────────────────────────────────────────────────────────────────────
// 1. INCLUDES
// ─────────────────────────────────────────────────────────────────────────────
#include <Wire.h>

// Dabble Bluetooth app (uses SoftwareSerial internally)
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>

// Enable optimized interrupt handling for Encoder library before including it.
// This uses direct ISR attachment instead of attachInterrupt(), saving time.
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// ─────────────────────────────────────────────────────────────────────────────
// 2. PIN DEFINITIONS (from e-Yantra botController v1.1 - DO NOT CHANGE)
// ─────────────────────────────────────────────────────────────────────────────

// Left Motor (Motor A on L298N)
#define ENA       6     // PWM speed control (Timer0)
#define IN1       A2    // Direction control 1
#define IN2       A3    // Direction control 2

// Right Motor (Motor B on L298N)
#define ENB       5     // PWM speed control (Timer0)
#define IN3       9     // Direction control 1
#define IN4       4     // Direction control 2

// PHYSICAL WIRING (verified by test): the encoder wires on the PCB are
// cross-wired relative to the motor labels. The LEFT motor's encoder is
// actually connected to pins 2,3 and the RIGHT motor's encoder to pins 7,8.
// We define them by PHYSICAL CONNECTION, then assign Encoder objects correctly.

// Left Motor's Encoder - physically on pins 2,3 (INT0/INT1 - reliable!)
#define ENC_LEFT_A    2     // Channel A (INT0) - hardware interrupt
#define ENC_LEFT_B    3     // Channel B (INT1) - hardware interrupt

// Right Motor's Encoder - physically on pins 7,8 (NO hardware interrupt)
// WARNING: Pins 7,8 use PCINT which drops ~96% of edges at motor speed.
// This encoder is UNRELIABLE for velocity measurement. The code uses only
// the left encoder (pins 2,3) for velocity and doubles it to estimate
// total robot velocity. The right encoder is kept for rough yaw estimation.
#define ENC_RIGHT_A   7     // Channel A (PCINT - unreliable at speed)
#define ENC_RIGHT_B   8     // Channel B (PCINT - unreliable at speed)

// MPU6050 I2C address
#define MPU_ADDR  0x68

// Peripherals
#define BUZZER_PIN  A1
#define SERVO_PIN   A0  // Reserved for future MG90s servo

// HC-05 Bluetooth module on SoftwareSerial (pins 12/13) via Dabble library
// Pins 10/11 share PCINT0 port with encoder pins 7/8 — causes conflict.
// Pins 12/13 are also PCINT0 but we try them since 10/11 didn't work.
// If still fails, HC-05 may need to go back on pins 0/1 without Dabble.
// HC-05 TX -> Arduino pin 12 (RX)
// HC-05 RX -> Arduino pin 13 (TX)
// USB Serial remains free for debug at 115200.

// ─────────────────────────────────────────────────────────────────────────────
// 3. HARDWARE CONSTANTS
// ─────────────────────────────────────────────────────────────────────────────

// Encoder specifications for N20 motors.
// IMPORTANT: These values vary by N20 variant. To calibrate:
//   1. Disconnect motors from control, print encoder1.read() in a loop
//   2. Manually rotate the OUTPUT SHAFT exactly one full turn
//   3. The reading / 4 = your ENCODER_CPR
//   4. ENCODER_CPR * 4 * GEAR_RATIO = counts per output shaft revolution
// Encoder ticks per output shaft revolution.
// From test data: left motor at PWM=60 for 3 sec gave ~21600 ticks.
// At ~70 RPM (300 RPM * 60/255), that's ~3.5 revs in 3 sec -> ~6170 ticks/rev.
// This suggests a higher CPR or gear ratio than the defaults.
// TO VERIFY: manually rotate output shaft exactly 1 full turn and read count.
// For now, the exact value doesn't matter for balance — PID gains absorb the units.
// It only matters if you convert to physical velocity (m/s).
#define TICKS_PER_REV   6170L   // Approximate, verify by manual rotation test

#define WHEEL_DIAMETER_M  0.043f  // 43mm typical N20 wheel, in meters
#define WHEEL_CIRCUMF_M   (3.14159f * WHEEL_DIAMETER_M)
#define WHEEL_BASE_M      0.10f   // Distance between wheels in meters (measure yours)

// Motor deadband: minimum PWM that overcomes stiction in N20 + L298N.
// To find: slowly increase PWM from 0 until wheel just starts turning.
#define MOTOR_DEADBAND  15    // Was 30 — lower to reduce jerkiness around zero

// Motor asymmetry calibration factors.
// To find: spin each motor at PWM=100, measure velocities, compute ratio.
// leftMotorScale  = reference_velocity / measured_left_velocity
// rightMotorScale = reference_velocity / measured_right_velocity
float leftMotorScale  = 1.0f;
float rightMotorScale = 1.0f;

// Set to true if right motor spins opposite direction for "forward" due to
// mirror mounting. VERIFIED: right motor needs inversion.
#define RIGHT_MOTOR_INVERTED  true

// Left encoder (pins 2,3) reads NEGATIVE when left motor drives forward.
// We invert in software so positive delta = forward.
#define LEFT_ENCODER_INVERTED  true

// Right encoder (pins 7,8) is UNRELIABLE (PCINT drops ~96% of edges).
// It is only used for rough yaw estimation when available.
#define RIGHT_ENCODER_INVERTED  false

// MPU6050 sensitivity at default ranges
#define GYRO_SENSITIVITY   131.0f   // LSB per deg/s at ±250 deg/s
#define ACCEL_SENSITIVITY  16384.0f // LSB per g at ±2g
#define RAD_TO_DEG         57.2958f

// Set to 1 or -1 depending on MPU6050 orientation on your chassis.
// If robot tilts forward and pitch_angle goes negative, set this to -1.
#define PITCH_SIGN  1

// ─────────────────────────────────────────────────────────────────────────────
// 4. TUNEABLE PID PARAMETERS (mutable for easy adjustment)
// ─────────────────────────────────────────────────────────────────────────────

// --- Inner Loop: Angle PID (100 Hz) ---
// Controls balance. Output = motor PWM command.
// Tuning: Start with Kp only. Increase until oscillation, back off 20%.
//         Add Kd to reduce oscillation. Ki is usually 0 or very small.
// Tuning log:
//   Kp=45, Kd=0.5  -> too jerky, too fast
//   Kp=25, Kd=0.8  -> MotCmd saturating (400-700), huge swings, no balance
//   Now using gyro rate for D-term (deg/s units), so Kd scale changes.
//   At 10° tilt:  P = 10*10 = 100 PWM (reasonable)
//   At 100°/s fall: D = 0.5*100 = 50 PWM braking (good)
//   LEARNING FROM OLD WORKING CODE:
//   Old code: Kp=45, Kd=0.5, cmd=-cmd, MAX_PWM=50
//   It used aggressive Kp but clamped output to ±50 PWM.
//   Effectively bang-bang for errors > 1°, proportional only near 0.
//   We now clamp output to ±60 and negate cmd like the old code.
//   With Kp=45: 1° error -> 45 PWM (good), 2° -> 90 -> clamped to 60.
float Kp_angle = 45.0f;    // Match old working code
float Ki_angle = 0.0f;     // Old code had Ki=0
float Kd_angle = 0.8f;     // Increased from 0.5 — more damping for arm inertia
                            // With gyro rate D-term, similar scale works

// --- Outer Loop: Velocity PID (20 Hz) ---
// Controls forward/backward motion. Output = angle setpoint offset (degrees).
// This creates the cascade: velocity PID -> target angle -> angle PID -> motors.
// CRITICAL: these must be MUCH smaller than angle gains. The outer loop should
// only gently nudge the target angle, not slam it around.
float Kp_vel = 0.025f;   // 5x increase from 0.005 — at 200 ticks: 0.025*200 = 5° (clamped to 3)
float Ki_vel = 0.01f;    // 5x increase from 0.002
float Kd_vel = 0.0f;

// --- Yaw PID (20 Hz) ---
// Controls turning via gyro Z. Output = PWM differential.
// Keep very low until balance works first.
float Kp_yaw = 1.5f;     // Increased — still rotating rightward from arm asymmetry
float Ki_yaw = 0.3f;     // Stronger integral to correct persistent rightward rotation
float Kd_yaw = 0.0f;

// Anti-windup integral limits
#define ANGLE_INTEGRAL_LIMIT  50.0f
#define VEL_INTEGRAL_LIMIT    10.0f
#define YAW_INTEGRAL_LIMIT    30.0f

// Velocity PID output limit: max angle offset in degrees
#define VEL_OUTPUT_LIMIT  3.0f

// Yaw PID output limit: max PWM differential
#define YAW_OUTPUT_LIMIT  30.0f   // Was 80 — reduced to prevent fighting balance

// ─────────────────────────────────────────────────────────────────────────────
// 5. FILTER & TIMING CONSTANTS
// ─────────────────────────────────────────────────────────────────────────────

// Complementary filter alpha: higher = more gyro trust (less noise, more drift)
// 0.96 is standard for self-balancing robots
#define COMP_ALPHA  0.96f

// Loop timing
#define INNER_LOOP_US   10000UL   // 100 Hz = 10 ms
#define OUTER_LOOP_MS   50UL      // 20 Hz  = 50 ms
#define DEBUG_PRINT_MS  500UL     // Debug output every 500 ms

// Safety thresholds
#define FALL_ANGLE      45.0f   // If |pitch| > this, robot has fallen - stop motors
#define RECOVERY_ANGLE  5.0f    // Must be within this to start/resume balancing

// ─────────────────────────────────────────────────────────────────────────────
// 6. SETPOINTS (change these to make the robot move)
// ─────────────────────────────────────────────────────────────────────────────

// Base target angle when standing still (small offset compensates for
// center-of-gravity not being exactly above the wheel axle).
// Positive = leaning forward. Tune by observing which way robot drifts.
float baseTargetAngle = -1.1f;  // Between -1.0 (forward drift) and -1.5 (backward drift)

// Forward velocity command. Units: average encoder ticks per 50ms window.
// 0 = stand still. Positive = forward. Start small (e.g., 5-10).
float velocity_setpoint = 0.0f;

// Yaw rate command. Units: degrees/second (from MPU6050 gyro Z axis).
// 0 = go straight. Positive = turn right (verify sign on your bot).
float yaw_rate_setpoint = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
// 7. GLOBAL STATE VARIABLES
// ─────────────────────────────────────────────────────────────────────────────

// --- Sensor raw data ---
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;

// --- Complementary filter state ---
float pitch_angle   = 0.0f;    // Estimated pitch in degrees
float gyro_rate     = 0.0f;    // Current gyro angular rate (deg/s) around pitch axis
float gyro_offset   = 0.0f;    // Calibrated gyro X-axis offset (raw units)
float gyro_offset_z = 0.0f;    // Calibrated gyro Z-axis offset (for yaw, raw units)

// --- Angle PID state ---
float angle_integral   = 0.0f;
float angle_prev_meas  = 0.0f;  // For derivative-on-measurement

// --- Velocity PID state ---
float vel_integral     = 0.0f;
float vel_prev_error   = 0.0f;

// --- Yaw PID state ---
float yaw_integral     = 0.0f;
float yaw_prev_error   = 0.0f;

// --- Encoder state ---
long enc_left_prev  = 0;
long enc_right_prev = 0;

// --- PID outputs (held between loops) ---
float motorCommand       = 0.0f;  // Angle PID output (base PWM)
float angleAdjustment    = 0.0f;  // Velocity PID output (angle offset)
float yawCorrection      = 0.0f;  // Yaw PID output (differential PWM)
float targetAngle        = 0.0f;  // Current target angle for inner loop

// --- Timing ---
unsigned long innerPrevUs  = 0;
unsigned long outerPrevMs  = 0;
unsigned long debugPrevMs  = 0;

// --- System flags ---
bool controlActive = false;

// ─────────────────────────────────────────────────────────────────────────────
// 8. OBJECT INSTANTIATIONS
// ─────────────────────────────────────────────────────────────────────────────

// Left motor encoder on pins 2,3 (INT0/INT1): RELIABLE, hardware interrupt
// Right motor encoder on pins 7,8 (PCINT): UNRELIABLE, drops most edges
Encoder encoderLeft(ENC_LEFT_A, ENC_LEFT_B);
Encoder encoderRight(ENC_RIGHT_A, ENC_RIGHT_B);

// Dabble movement state
unsigned long lastMoveCmdTime = 0;
#define MOVE_CMD_TIMEOUT  300UL     // Stop if no command for 300ms

// ─────────────────────────────────────────────────────────────────────────────
// 9. HELPER FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────

void buzzerBeep(uint16_t freq, uint16_t durationMs) {
  tone(BUZZER_PIN, freq, durationMs);
}

void resetPIDState() {
  angle_integral  = 0.0f;
  angle_prev_meas = pitch_angle;
  vel_integral    = 0.0f;
  vel_prev_error  = 0.0f;
  yaw_integral    = 0.0f;
  yaw_prev_error  = 0.0f;
  motorCommand    = 0.0f;
  angleAdjustment = 0.0f;
  yawCorrection   = 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. MPU6050 FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────

void initMPU() {
  // Wake up MPU6050 (clear sleep bit in power management register)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up (CLKSEL=0, internal 8MHz oscillator)
  Wire.endTransmission(true);

  // Set gyroscope range to ±250 deg/s (sensitivity = 131 LSB/deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x00);  // FS_SEL=0: ±250 deg/s
  Wire.endTransmission(true);

  // Set accelerometer range to ±2g (sensitivity = 16384 LSB/g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // AFS_SEL=0: ±2g
  Wire.endTransmission(true);

  // Set Digital Low Pass Filter to ~44 Hz bandwidth, ~4.9 ms delay.
  // Good compromise: filters vibration noise but keeps latency < 5ms.
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // DLPF_CFG=3: accel BW 44Hz, gyro BW 42Hz
  Wire.endTransmission(true);
}

void readMPUData() {
  // Burst-read 14 bytes: accel XYZ (6) + temp (2) + gyro XYZ (6)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  raw_ax = (Wire.read() << 8) | Wire.read();
  raw_ay = (Wire.read() << 8) | Wire.read();
  raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Skip temperature
  raw_gx = (Wire.read() << 8) | Wire.read();
  raw_gy = (Wire.read() << 8) | Wire.read();
  raw_gz = (Wire.read() << 8) | Wire.read();
}

void calibrateGyro() {
  // Average 2000 gyro samples to find DC offset.
  // Robot MUST be completely still during this (~4 seconds).
  Serial.println(F("Calibrating gyro - keep robot still..."));

  long sumGx = 0;
  long sumGz = 0;
  for (int i = 0; i < 2000; i++) {
    readMPUData();
    sumGx += raw_gx;
    sumGz += raw_gz;
    if (i % 400 == 0) Serial.print(F("."));
    delay(2);  // ~2ms per sample = ~4 seconds total
  }
  gyro_offset   = (float)sumGx / 2000.0f;
  gyro_offset_z = (float)sumGz / 2000.0f;

  Serial.println();
  Serial.print(F("Gyro X offset: "));
  Serial.println(gyro_offset, 2);
  Serial.print(F("Gyro Z offset: "));
  Serial.println(gyro_offset_z, 2);
}

void updateComplementaryFilter(float dt) {
  // Read fresh sensor data
  readMPUData();

  // Accelerometer-based angle (reliable long-term, noisy short-term)
  // atan2(ay, az) gives pitch angle assuming:
  //   - MPU mounted flat on chassis
  //   - Y axis points along forward direction of robot
  //   - Z axis points upward
  // ADJUST the axis pair if your MPU is oriented differently!
  float accelAngle = atan2((float)raw_ay, (float)raw_az) * RAD_TO_DEG;

  // Gyroscope rate (reliable short-term, drifts long-term)
  // Subtract calibrated offset, convert to deg/s
  gyro_rate = ((float)raw_gx - gyro_offset) / GYRO_SENSITIVITY;

  // Apply pitch sign correction for MPU orientation
  accelAngle *= PITCH_SIGN;
  gyro_rate  *= PITCH_SIGN;

  // Complementary filter: blend gyro integration with accel angle
  // High-pass on gyro (short-term trust) + low-pass on accel (long-term trust)
  pitch_angle = COMP_ALPHA * (pitch_angle + gyro_rate * dt)
              + (1.0f - COMP_ALPHA) * accelAngle;
}

// ─────────────────────────────────────────────────────────────────────────────
// 11. MOTOR CONTROL FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────

void stopMotors() {
  // Use digitalWrite for true zero (avoid Timer0 glitch on pins 5,6)
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setMotorLeft(int pwm) {
  // pwm: -255 to +255. Positive = robot forward.
  pwm = constrain(pwm, -255, 255);
  int absPwm = abs(pwm);

  if (absPwm == 0) {
    // Coast (both LOW)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(ENA, LOW);
    return;
  }

  // No dead-zone — let PID output go directly to motor.
  // L298N won't move at very low PWM anyway (stiction), but the PID
  // integral will keep increasing until the command is large enough.
  if (pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, absPwm);
}

void setMotorRight(int pwm) {
  // pwm: -255 to +255. Positive = robot forward.
  pwm = constrain(pwm, -255, 255);
  int absPwm = abs(pwm);

  if (absPwm == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(ENB, LOW);
    return;
  }

  // RIGHT_MOTOR_INVERTED: because the right motor is mirror-mounted,
  // its "forward" electrical direction produces backward robot motion.
  // We invert the direction logic here so positive pwm = robot forward.
  #if RIGHT_MOTOR_INVERTED
    if (pwm > 0) {
      // Robot forward: electrically "reverse" for right motor
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      // Robot backward: electrically "forward" for right motor
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
  #else
    if (pwm > 0) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
  #endif

  analogWrite(ENB, absPwm);
}

void setMotors(int leftPwm, int rightPwm) {
  setMotorLeft(leftPwm);
  setMotorRight(rightPwm);
}

// ─────────────────────────────────────────────────────────────────────────────
// 12. PID COMPUTATION FUNCTIONS
// ─────────────────────────────────────────────────────────────────────────────

float innerLoopAngleControl(float currentAngle, float target, float dt) {
  // Inner loop PID: keeps robot upright.
  // Input: pitch angle (degrees)
  // Output: base motor PWM command

  float error = target - currentAngle;

  // Integral with anti-windup clamping
  angle_integral += error * dt;
  angle_integral = constrain(angle_integral, -ANGLE_INTEGRAL_LIMIT, ANGLE_INTEGRAL_LIMIT);

  // D-term: use GYRO RATE directly instead of differentiating angle numerically.
  // gyro_rate (deg/s) is a clean, high-bandwidth measurement of angular velocity.
  // When the robot tilts forward (error positive), gyro_rate is negative (falling),
  // so we use -gyro_rate to get a braking force proportional to fall speed.
  // This is MUCH more effective than numerical differentiation of noisy angle.
  float derivative = -gyro_rate;

  // PID output
  float output = Kp_angle * error
               + Ki_angle * angle_integral
               + Kd_angle * derivative;

  return output;
}

float outerLoopVelocityControl(float currentVelocity, float target, float dt) {
  // Outer loop PID: controls forward/backward motion.
  // Input: average wheel velocity (encoder ticks per 50ms window)
  // Output: angle setpoint offset (degrees) fed to inner loop.

  float error = target - currentVelocity;

  // Integral with anti-windup
  vel_integral += error * dt;
  vel_integral = constrain(vel_integral, -VEL_INTEGRAL_LIMIT, VEL_INTEGRAL_LIMIT);

  // Derivative
  float derivative = (error - vel_prev_error) / dt;
  vel_prev_error = error;

  float output = Kp_vel * error
               + Ki_vel * vel_integral
               + Kd_vel * derivative;

  // Limit angle offset to prevent excessive lean
  return constrain(output, -VEL_OUTPUT_LIMIT, VEL_OUTPUT_LIMIT);
}

float computeYawCorrection(float currentYawRate, float target, float dt) {
  // Yaw PD controller: turns the robot via differential wheel speed.
  // Input: differential encoder ticks per 50ms window
  // Output: PWM differential (added to left, subtracted from right)

  float error = target - currentYawRate;

  yaw_integral += error * dt;
  yaw_integral = constrain(yaw_integral, -YAW_INTEGRAL_LIMIT, YAW_INTEGRAL_LIMIT);

  float derivative = (error - yaw_prev_error) / dt;
  yaw_prev_error = error;

  float output = Kp_yaw * error
               + Ki_yaw * yaw_integral
               + Kd_yaw * derivative;

  return constrain(output, -YAW_OUTPUT_LIMIT, YAW_OUTPUT_LIMIT);
}

// ─────────────────────────────────────────────────────────────────────────────
// 13. DEBUG TELEMETRY
// ─────────────────────────────────────────────────────────────────────────────

void printDebug() {
  // Output to Serial (goes to USB or HC-05 BT on pins 0/1).
  // NOTE: Arduino AVR snprintf doesn't support floats, so use Serial.print.
  Serial.print(F("P:"));  Serial.print(pitch_angle, 1);
  Serial.print(F(" T:")); Serial.print(targetAngle, 1);
  Serial.print(F(" M:")); Serial.print(motorCommand, 0);
  Serial.print(F(" G:")); Serial.println(gyro_rate, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// 14. DABBLE GAMEPAD PROCESSING
// ─────────────────────────────────────────────────────────────────────────────

void readDabble() {
  // Process Dabble app data.
  Dabble.processInput();

  bool anyPressed = false;

  bool up = GamePad.isUpPressed();
  bool dn = GamePad.isDownPressed();
  bool lt = GamePad.isLeftPressed();
  bool rt = GamePad.isRightPressed();

  // Debug: print button states
  if (up || dn || lt || rt) {
    Serial.print(F("DAB: "));
    if (up) Serial.print(F("UP "));
    if (dn) Serial.print(F("DN "));
    if (lt) Serial.print(F("LT "));
    if (rt) Serial.print(F("RT "));
    Serial.println();
  }

  // D-pad buttons
  if (up) {
    velocity_setpoint = 10.0f;
    anyPressed = true;
  }
  else if (dn) {
    velocity_setpoint = -10.0f;
    anyPressed = true;
  }
  else {
    velocity_setpoint = 0.0f;
  }

  if (lt) {
    yaw_rate_setpoint = -5.0f;
    anyPressed = true;
  }
  else if (rt) {
    yaw_rate_setpoint = 5.0f;
    anyPressed = true;
  }
  else {
    yaw_rate_setpoint = 0.0f;
  }

  // Triangle/Circle/Cross/Square for PID quick-adjust
  // Debounced: only triggers once per press using static flags
  static bool triPrev = false, crsPrev = false, cirPrev = false, sqPrev = false;

  bool tri = GamePad.isTrianglePressed();
  bool crs = GamePad.isCrossPressed();
  bool cir = GamePad.isCirclePressed();
  bool sq  = GamePad.isSquarePressed();

  if (tri && !triPrev) Kp_angle += 1.0f;
  if (crs && !crsPrev) Kp_angle -= 1.0f;
  if (cir && !cirPrev) baseTargetAngle += 0.1f;
  if (sq  && !sqPrev)  baseTargetAngle -= 0.1f;

  triPrev = tri; crsPrev = crs; cirPrev = cir; sqPrev = sq;

  if (anyPressed) lastMoveCmdTime = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// 15. SAFETY FUNCTIONS

// ─────────────────────────────────────────────────────────────────────────────

void waitForUpright() {
  // Block until robot is held upright (within RECOVERY_ANGLE of vertical).
  // Continuously updates pitch estimate so filter stays primed.
  Serial.println(F("Hold robot upright (within 5 deg)..."));

  while (true) {
    readMPUData();
    float angle = atan2((float)raw_ay, (float)raw_az) * RAD_TO_DEG * PITCH_SIGN;

    if (abs(angle) < RECOVERY_ANGLE) {
      // Prime the complementary filter with current accel angle
      pitch_angle = angle;
      angle_prev_meas = angle;
      Serial.println(F("Upright detected!"));
      buzzerBeep(1500, 200);
      delay(300);
      return;
    }

    // Print current angle for feedback
    Serial.print(F("Angle: "));
    Serial.print(angle, 1);
    Serial.println(F(" deg"));

    // Blink buzzer to indicate waiting
    if ((millis() / 500) % 2 == 0) {
      buzzerBeep(800, 50);
    }
    delay(200);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 16. SETUP
// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  // --- Serial for USB debug ---
  Serial.begin(115200);
  Serial.println(F("\n=== SELF-BALANCING ROBOT INIT ==="));

  // --- Dabble Bluetooth (HC-05 on SoftwareSerial pins 10,11) ---
  Dabble.begin(9600, 11, 10);  // baud, RX pin, TX pin (try swapped)

  // --- Motor pins ---
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotors();

  // --- Buzzer ---
  pinMode(BUZZER_PIN, OUTPUT);

  // --- I2C at 400 kHz (fast mode) ---
  // 14-byte MPU read takes ~0.35ms at 400kHz vs ~1.4ms at 100kHz.
  // Critical for staying within the 10ms inner loop budget.
  Wire.begin();
  Wire.setClock(400000);

  // --- Initialize MPU6050 ---
  initMPU();
  delay(100);  // Let MPU stabilize after wake-up

  // --- Gyro calibration ---
  // Robot MUST be completely still for ~4 seconds
  buzzerBeep(1000, 300);
  delay(500);
  calibrateGyro();

  // --- Signal calibration complete ---
  buzzerBeep(2000, 200);
  delay(200);
  buzzerBeep(2000, 200);
  delay(500);

  // --- Wait for robot to be held upright ---
  waitForUpright();

  // --- Reset encoders to zero ---
  encoderLeft.write(0);
  encoderRight.write(0);
  enc_left_prev  = 0;
  enc_right_prev = 0;

  // --- Reset PID state ---
  resetPIDState();

  // --- Initialize timing ---
  innerPrevUs = micros();
  outerPrevMs = millis();
  debugPrevMs = millis();

  // --- Ready signal: two beeps BEFORE enabling control ---
  // CRITICAL: beeps must happen BEFORE controlActive=true, because
  // delay() blocks the main loop. If control is active during delay(),
  // the robot falls with no PID running for 500ms.
  buzzerBeep(1200, 200);
  delay(300);
  buzzerBeep(1200, 200);
  delay(300);

  Serial.println(F("=== ROBOT READY - BALANCING ==="));

  // --- Enable control LAST, right before loop() starts ---
  controlActive = true;

}

// ─────────────────────────────────────────────────────────────────────────────
// 17. MAIN LOOP
// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  unsigned long nowUs = micros();
  unsigned long nowMs = millis();

  // =========================================================================
  // INNER LOOP: 100 Hz (every 10 ms) - Angle Stabilization
  // =========================================================================
  if (nowUs - innerPrevUs >= INNER_LOOP_US) {
    float dt = (float)(nowUs - innerPrevUs) / 1000000.0f;  // Seconds
    innerPrevUs = nowUs;

    // 1. Update pitch angle via complementary filter
    updateComplementaryFilter(dt);

    // 2. Fall detection
    if (abs(pitch_angle) > FALL_ANGLE) {
      if (controlActive) {
        // Robot has fallen over - emergency stop
        stopMotors();
        controlActive = false;
        resetPIDState();

        // Alarm beeps
        for (int i = 0; i < 3; i++) {
          buzzerBeep(1000, 150);
          delay(200);
        }
        Serial.println(F("FALLEN! Hold upright to resume."));
      }

      // Wait for recovery: keep updating pitch so filter stays accurate
      // but don't run motors
      if (abs(pitch_angle) < RECOVERY_ANGLE) {
        // Robot is upright again - prepare to resume
        Serial.println(F("Upright - resuming in 1s..."));
        delay(1000);

        // Re-read angle to get fresh estimate
        for (int i = 0; i < 50; i++) {
          readMPUData();
          float accelAngle = atan2((float)raw_ay, (float)raw_az) * RAD_TO_DEG * PITCH_SIGN;
          float gRate = ((float)raw_gx - gyro_offset) / GYRO_SENSITIVITY * PITCH_SIGN;
          pitch_angle = COMP_ALPHA * (pitch_angle + gRate * 0.002f) + (1.0f - COMP_ALPHA) * accelAngle;
          delay(2);
        }

        resetPIDState();
        encoderLeft.write(0);
        encoderRight.write(0);
        enc_left_prev = 0;
        enc_right_prev = 0;

        controlActive = true;
        buzzerBeep(1500, 200);
        Serial.println(F("Balancing resumed."));
      }
      return;  // Skip control when fallen
    }

    // 3. Compute angle PID (inner loop)
    if (controlActive) {
      // targetAngle is updated by the outer loop's velocity PID
      motorCommand = innerLoopAngleControl(pitch_angle, targetAngle, dt);

      // NEGATE: the PID convention (positive error -> positive output) is
      // opposite to the motor convention needed for balance. The old working
      // code used cmd = -cmd. Without negation, when the robot tilts forward
      // (positive pitch, negative error), the PID outputs a negative command
      // which drives motors backward — pushing the robot MORE forward instead
      // of catching the fall.
      float cmd = -motorCommand;

      // CLAMP to low PWM range. The old working code used max PWM of 50.
      // N20 300RPM motors through L298N need gentle corrections, not full blast.
      // Higher PWM causes overshoot and oscillation.
      int leftPwm  = constrain((int)(cmd + yawCorrection), -80, 80);
      int rightPwm = constrain((int)(cmd - yawCorrection), -80, 80);

      setMotors(leftPwm, rightPwm);
    }
  }

  // =========================================================================
  // OUTER LOOP: 20 Hz (every 50 ms) - Velocity & Yaw Control
  // =========================================================================
  if (nowMs - outerPrevMs >= OUTER_LOOP_MS) {
    float dtOuter = (float)(nowMs - outerPrevMs) / 1000.0f;  // Seconds
    outerPrevMs = nowMs;

    // 1. Read encoder positions and compute DELTAS (not accumulated position!)
    //    This is the key to avoiding encoder drift.
    //
    //    HARDWARE LIMITATION: Right encoder (pins 7,8) uses PCINT which drops
    //    ~96% of edges at motor speed. Only the LEFT encoder (pins 2,3) with
    //    hardware interrupts is reliable. We use LEFT encoder only for forward
    //    velocity (doubled to estimate total), and ignore right encoder for now.
    //    Yaw control uses the gyro gz axis instead of encoder differential.
    long encL = encoderLeft.read();

    // Apply encoder inversion: left encoder reads negative for forward motion
    #if LEFT_ENCODER_INVERTED
      long deltaL = -(encL - enc_left_prev);
    #else
      long deltaL = encL - enc_left_prev;
    #endif

    enc_left_prev = encL;

    // 2. Compute chassis velocity from LEFT encoder only
    //    Since both motors get the same PWM command (minus yaw differential),
    //    we assume left wheel velocity ≈ right wheel velocity.
    //    Forward velocity ≈ left wheel velocity (no need to double/average
    //    since PID gains absorb the scale factor).
    float vAvg = (float)deltaL * leftMotorScale;

    // 3. Yaw rate: without a reliable second encoder, we use the MPU6050
    //    gz (yaw) gyro axis for turning feedback instead of encoder differential.
    //    gyro_gz is in raw units; divide by GYRO_SENSITIVITY for deg/s.
    float yawRate = ((float)raw_gz - gyro_offset_z) / GYRO_SENSITIVITY;

    // 4. Velocity PID -> angle offset
    angleAdjustment = outerLoopVelocityControl(vAvg, velocity_setpoint, dtOuter);

    // 5. Update target angle for inner loop
    //    baseTargetAngle: physical center of balance
    //    angleAdjustment: lean forward/backward to move
    targetAngle = baseTargetAngle + angleAdjustment;

    // 6. Yaw PID -> differential PWM (using gyro yaw rate instead of encoders)
    yawCorrection = computeYawCorrection(yawRate, yaw_rate_setpoint, dtOuter);
  }

  // =========================================================================
  // DEBUG OUTPUT: Every 500 ms (via USB Serial, BT is on separate pins now)
  // =========================================================================
  if (nowMs - debugPrevMs >= DEBUG_PRINT_MS) {
    debugPrevMs = nowMs;
    if (controlActive) {
      printDebug();
    }
  }

  // =========================================================================
  // DABBLE: Read gamepad input at outer loop rate (20 Hz)
  // Running every iteration overwhelms SoftwareSerial and hurts balance.
  // =========================================================================
  static unsigned long dabblePrevMs = 0;
  if (nowMs - dabblePrevMs >= OUTER_LOOP_MS) {
    dabblePrevMs = nowMs;
    readDabble();
  }
}
