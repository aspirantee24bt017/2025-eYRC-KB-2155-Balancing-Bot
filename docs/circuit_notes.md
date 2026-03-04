# Circuit & Wiring Notes — Krishi Balancer KB#2155

## Pin Map (Arduino Nano)

### Motor Driver (L298N — botController v1.1 PCB)
| Arduino Pin | Function |
|-------------|----------|
| D6 (PWM) | ENA — Left motor speed |
| A2 | IN1 — Left motor direction |
| A3 | IN2 — Left motor direction |
| D5 (PWM) | ENB — Right motor speed |
| D9 | IN3 — Right motor direction |
| D4 | IN4 — Right motor direction |

### Encoders
| Arduino Pin | Function | Notes |
|-------------|----------|-------|
| D2 | Left encoder channel A | INT0 — hardware interrupt, RELIABLE |
| D3 | Left encoder channel B | INT1 — hardware interrupt, RELIABLE |
| D7 | Right encoder channel A | PCINT — unreliable at motor speed |
| D8 | Right encoder channel B | PCINT — unreliable at motor speed |

> **Important:** Right encoder (pins 7/8) drops ~96% of edges at speed due to PCINT limitations.
> Yaw control uses gyro Z axis instead of right encoder differential.

### IMU
| Arduino Pin | Function |
|-------------|----------|
| A4 (SDA) | MPU6050 data |
| A5 (SCL) | MPU6050 clock |

I2C speed set to **400 kHz** (fast mode). Reduces 14-byte read time from ~1.4ms to ~0.35ms.

### Peripherals
| Arduino Pin | Function |
|-------------|----------|
| A1 | Buzzer |
| A0 | Servo (reserved, not used) |
| D12 | HC-05 TX → Arduino RX (SoftwareSerial) |
| D13 | HC-05 RX → Arduino TX (SoftwareSerial) |

---

## Gyro Calibration Procedure

On every power-up, the code auto-calibrates (2000 samples, ~4 seconds):
1. Place robot on flat surface completely still
2. Wait for single beep → calibration starts
3. Do NOT touch or move robot
4. Calibration stores `gyro_offset` and `gyro_offset_z`

To manually verify calibration values, open Serial Monitor at **115200 baud** and look for:
```
Gyro X offset: <value>
Gyro Z offset: <value>
```

---

## Encoder Tick Calibration

To find your exact `TICKS_PER_REV`:
1. Disconnect motors from control loop
2. Print `encoderLeft.read()` in a serial loop
3. Manually rotate the OUTPUT SHAFT exactly one full turn
4. The count / 4 = encoder CPR
5. CPR × gear_ratio = TICKS_PER_REV

Current value: **6170** (approximate, verified empirically)

---

## Motor Asymmetry Calibration

To calibrate `leftMotorScale` / `rightMotorScale`:
1. Spin each motor independently at PWM = 100
2. Measure RPM (or count encoder ticks over 1 second)
3. `scale = reference_velocity / measured_velocity`
4. Update constants in PID.ino

---

## Known Issues

- **PCINT conflict:** Bluetooth HC-05 on pins 10/11 originally conflicted with encoder PCINT port.
  Resolved by moving HC-05 to pins 12/13.
- **Right motor polarity:** Mirror-mounted on chassis. `RIGHT_MOTOR_INVERTED true` handles this in firmware.
- **baseTargetAngle:** Currently set to -1.1°. This compensates for the robot's center of gravity
  not being exactly above the wheel axle. Tune this if robot drifts forward/backward at rest.
