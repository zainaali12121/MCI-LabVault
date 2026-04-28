# Self-Balancing Robot — STM32F3 Discovery

Two-wheeled self-balancing robot using an STM32F3 Discovery board, PID control, and IMU sensor fusion.

## Requirements

- STM32F3 Discovery board
- STM32CubeMX Software
- Keyestudio self-balancing robot kit (motors + H-bridge)
- LSM303 accelerometer (I2C) + I3G4250D gyroscope (SPI) — onboard the Discovery

## Wiring

| Signal       | Pin |
|--------------|-----|
| Motor A DIR1 | PB12|
| Motor A DIR2 | PD8 |
| Motor B DIR1 | PD6 |
| Motor B DIR2 | PD7 |
| Motor PWM CH1| TIM3 CH1 |
| Motor PWM CH2| TIM3 CH2 |
| Gyro CS      | PE3|

## Build & Flash

1. Clone/open the project in STM32CubeIDE.
2. Build (`Ctrl+B`).
3. Flash via ST-Link (`Run > Debug` or `Run > Run`).

## First Run

Place the robot upright and stationary before powering on. At startup, `OffsetLSM()` collects 60 samples (~3 seconds) to auto-calibrate the IMU. Moving the robot during this window will corrupt the calibration.

## Tuning

All tunable parameters are at the top of `main.c`:

```c
float Kp       = 57.0f;   // increase for stronger correction
float Ki       = 0.0f;    // enable to fix steady-state lean
float Kd       = 1.0f;    // increase to reduce oscillation
float setpoint = 2.0f;    // adjust if robot drifts forward/back at rest
```

## Motor Test

To verify motor wiring before running the control loop, uncomment the test block inside `while(1)` in `main.c` and comment out the TIM2 interrupt start.

## Known Limitations

-  the robot may exhibit a small steady-state lean on uneven surfaces, setpoint is -2.0f
- Calibration assumes a flat surface at startup.
- Recovery fails beyond ±90° tilt (motors cut off by design).
