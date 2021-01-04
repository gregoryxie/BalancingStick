
# BalancingStick

## Overview
Double reaction wheel balancing stick, using 2 pairs of DJI's M2006/C610 motor controller and ESC combo. 

![It balances!](imgs/balancing.jpg?raw=true)

Video of the stick balancing: https://www.youtube.com/watch?v=M8iMUHhxG7U

Inspired by Mike Rouleau's balancing stick.

## Control Loop

The Teensy runs a full state feedback controller, initially designed using LQR and then tuned. The states are augmented with additional open loop integrators on the velocities of both flywheels. Since the reference input is 0, the flywheel velocities should also be 0 at steady state (open loop integrator drives error to 0). This allows the controller to compensate for the imperfect zeroing of the IMU, since a 0 flywheel velocity means that the stick is perfectly balanced.

There are also first order low pass filters on the roll rate and pitch rate, as the raw measurements from the IMU were quite noisy. 

## CAD

Solidworks models for the stick can be found on [GrabCAD](https://grabcad.com/library/self-balancing-stick-2). 