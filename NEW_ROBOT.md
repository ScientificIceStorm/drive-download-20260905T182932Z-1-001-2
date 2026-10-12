# New robot

## Controls

| Control | Action |
| --- | --- |
| Left stick Y | Drive forward/backward |
| Right stick X | Turn |
| R1 / R2 | Lift up / down; release to hold |
| L1 / L2 | Port A off / on (open / close, depending on tubing) |
| Up / Down | Port H on / off |
| X or screen Run | Selected auton; currently empty |

## Ports

| Device | Ports |
| --- | --- |
| Left drivetrain | 9, 20, -19 |
| Right drivetrain | 10, 18, -8 |
| Lift | 5, -7 |
| IMU | 11 |
| Forward/backward tracking wheel | -17, centered offset 0 |
| Claw solenoid | A |
| Other solenoid | H |
| Radio | 1, reserved; no motor/sensor declaration |

Negative motor ports reverse the commanded direction. Lift motor 7 is reversed relative to 5; check that both help the lift rather than fight each other. The four large drive motors are unreversed as supplied in the new port map; verify direction with the wheels off the ground before driving.

Both solenoid outputs start false. Physical positions depend on valve/tubing connections. Keep clear at startup.

## Coordinates

The screen, helpers, and native LemLib commands use X positive to the right and Y positive forward from the initial heading. Heading is 0 forward, 90 right, 180 backward, 270 left. These are fixed starting-frame coordinates, not directions that rotate with the robot.

There is no coordinate mirroring. One forward/backward tracking wheel cannot measure sideways sliding.

## Measurements still needed

`include/robot_config.hpp` uses the confirmed 2.75-inch drive wheels and 450 wheel RPM. Track width is still a 10-inch placeholder; measure left/right wheel-center spacing before tuning auton. The tracking wheel uses the previous VEX 2-inch wheel (LemLib diameter 2.125 inches), direct drive, reversed sensor direction, and centered offset 0. PID settings are baseline values, not tuned for the new robot.

The 5.5W motors on 8 and 19 are configured as 200 RPM. The four 11W motors use blue 600 RPM cartridges. External gearing must give compatible wheel speeds: 600 RPM motors geared to 450 wheel RPM and 200 RPM motors geared to 450 wheel RPM. Software port reversal alone does not match different gear ratios.

After upload, start still for IMU calibration. Push forward 12 inches: Y should increase by about 12. Rotate clockwise 90 degrees: heading should increase by about 90. Start the lift physically lowered before using `lift_to`, since startup zeros its encoders.

The old auton is backed up in `/Users/albertzhang/Documents/Codex/2026-09-05/bru/old-bot-backup.iX7JyI`; the new `auton_steps()` only sets the starting pose and does not move mechanisms.
