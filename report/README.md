# PID and Cascade Position Control in Simulink

A Simulink model from an Industrial Control course project. It compares three ways of closing a position loop around the same plant: no controller, a single PID controller, and a cascade controller with a PID position loop wrapped around a proportional inner loop.

All three loops sit in one model with their own reference and their own scope, so the responses can be run and compared side by side.

The repository also contains an Arduino sketch that runs a PID position controller on a real motor with an encoder.

## Repository contents

```
.
├── PID.slx                                   Simulink model, saved in MATLAB R2016b
├── arduino/
│   └── pid_position_control/
│       └── pid_position_control.ino          Arduino PID position controller
├── docs/
│   └── report.pdf                            Full written report
└── README.md
```

## Requirements

MATLAB with Simulink. The model was saved in R2016b, so R2016b or newer is needed. Every block comes from the standard Simulink libraries, so no extra toolbox should be required.

For the Arduino sketch, the Arduino IDE, a board with two interrupt pins (pins 2 and 3 on an Uno), a DC motor with a quadrature encoder and a motor driver with two direction inputs and a PWM input.

## How to run

Open `PID.slx`, press Run, then double-click each Scope block to see the position response. From the MATLAB command window the same thing is:

```matlab
open_system('PID');
sim('PID');
```

The simulation runs from 0 to 10 s with an automatically chosen variable-step solver.

## The model

The plant is a second-order transfer function followed by an integrator, which turns the output into a position. This is the typical shape of a DC motor position servo.

```
G(s) = 1.2 / (0.00077 s^2 + 0.0539 s + 1.441)     followed by     1/s
```

The block has a DC gain of 0.833, a natural frequency of 43.3 rad/s and a damping ratio of 0.81.

| Loop | Structure |
| --- | --- |
| Without controller | Reference, sum, plant, integrator, with the position fed back through a unity path |
| PID controller | Reference, sum, PID Controller2, plant, integrator, with unity feedback |
| Cascade controller | Reference, sum, PID Controller1, inner sum, gain of 0.6, plant, integrator. The plant output closes the inner loop and the position closes the outer loop |

Both PID blocks use the parallel form with a filtered derivative, N = 100, and have no output limit or anti-windup.

| Parameter | PID Controller1 (cascade) | PID Controller2 (single loop) |
| --- | --- | --- |
| Kp | 63 | 50.4 |
| Ki | 6300/6 = 1050 | 50.4 × 14 = 705.6 |
| Kd | 0.04 × 63 = 2.52 | 50.4 / 56 = 0.9 |

The gains of PID Controller2 match the closed-loop Ziegler–Nichols rules, with an ultimate gain of about 84 and an oscillation period of about 1/7 s. The model does not say how the gains were chosen, so that match is an inference from the numbers.

## Results

Step response of the position to a unit step reference.

| Measure | Without controller | Single-loop PID | Cascade |
| --- | --- | --- | --- |
| Overshoot | none | 74.7% | 26.6% |
| Peak time | not applicable | 0.086 s | 0.169 s |
| Rise time, 10 to 90 percent | 2.55 s | 0.028 s | 0.031 s |
| Settling time, 2 percent band | 4.59 s | 0.484 s | 0.488 s |
| Steady-state error | 0 | 0 | 0 |

Both controllers settle about nine to ten times faster than the loop with no controller. The cascade loop does it with roughly a third of the overshoot of the single PID loop.

These figures were calculated outside Simulink from the same transfer functions and gains, using a unit step, and checked with two independent methods. The reference blocks in the model are pulse generators, so the traces on your scopes may differ from these numbers.

## Arduino implementation

`arduino/pid_position_control/pid_position_control.ino` runs a discrete PID controller every 100 ms. It reads the encoder through two interrupts, converts the count to an angle, and drives the motor with a PWM signal and two direction pins. The target is 100 degrees, and the angle is printed over the serial port at 9600 baud.

| Pin | Use |
| --- | --- |
| 2 | Encoder channel A, interrupt |
| 3 | Encoder channel B, interrupt |
| 4 | Motor direction 1 |
| 5 | Motor direction 2 |
| 11 | PWM output to the motor driver |

The gains are Kp 0.32, Ki 0.1 and Kd 0.3 and are meant to be adjusted by trial and error. They have nothing to do with the Simulink gains, because here the error is in degrees, the output is a PWM value from 0 to 255 and the loop runs at about 10 Hz.

The sketch corrects a bug in an earlier version, where the motor direction came from the sign of the error but the drive strength came from the absolute value of the PID output. That turned the braking action of the derivative term into a push toward the target. The direction now follows the sign of the PID output. The integral term is also clamped to avoid windup, and the encoder count shared with the interrupt routine is declared `volatile`.

Before using it, set `DEG_PER_COUNT` for your encoder. The sketch counts every edge of both channels, so the value is 360 divided by four times the encoder's pulses per revolution, for example 0.9 for a 100 pulse encoder. To check it, disconnect the motor supply, turn the shaft by hand through one revolution and confirm that the printed angle changes by 360. The sketch has not been tested on hardware, so try it first with the motor unloaded and at a reduced supply voltage.

## Limitations

The model is an ideal linear system with no actuator limit, load disturbance, sensor noise or friction. Because the controllers have no output limit, the control signal jumps to about 140 in the single loop and about 189 in the cascade loop at the instant a unit step arrives, which a real drive could not deliver. Adding saturation with anti-windup and retuning both controllers with the same method would be the natural next steps.

## Author

Created for the Industrial Control course by Hossein Mosaffa.
