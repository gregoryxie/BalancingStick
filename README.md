
# BalancingStick

## Overview
Double reaction wheel balancing stick, using 2 pairs of DJI's M2006/C610 motor controller and ESC combo. 

![It balances!](imgs/balancing_lower_res.jpg?raw=true)

Video of the stick balancing: https://youtu.be/Hc7ZZhGV2LI

Inspired by Mike Rouleau's balancing stick.

## Control Loop

The Teensy runs a full state feedback controller, initially designed using LQR and then tuned, with original states [pitch, pitch rate, pitch flywheel velocity, roll, roll rate, roll flywheel velocity]'.

The states are augmented with additional open loop integrators on the velocities of both flywheels. Since the reference input is 0, the flywheel velocities should also be 0 at steady state (open loop integrator drives error to 0). This is desirable for disturbance rejection, as control effort can be exerted equally in both rotation directions.

The wheel velocities conveniently act as integrators on the torque applied, which result in zero steady state commanded torque. This means that in theory, the stick will still balance if the IMU zero is imperfect, with or without the additional integrator on the flywheel velocity. However, this is not the case in real life. Later testing showed that system requires the additional integrator on the wheel velocity to stabilize. I believe this is because the coupling terms that were ignored in the model are proportional to wheel velocity. 

There are also first order low pass filters on the roll rate and pitch rate, as the raw measurements from the IMU were quite noisy. 

More information can be found on my [website](http://gregoryxie.com/). 

## CAD

Solidworks models for the stick can be found on [GrabCAD](https://grabcad.com/library/self-balancing-stick-2). 