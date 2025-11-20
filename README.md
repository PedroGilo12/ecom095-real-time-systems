# Final Project — Real-Time Systems
Implementation of the Bug 1 Algorithm with Position Estimation Using an Accelerometer
Project Description

This project implements a real-time embedded navigation system based on the classical Bug 1 algorithm. The goal is to guide a mobile robot from a starting point to a target location while avoiding obstacles detected locally.

As part of the Real-Time Systems course requirements, the system integrates several concepts covered in class, including:

- Periodic and sporadic tasks
- Deadlines and priority assignments
- Inter-task communication
- Timing and scheduling constraints
- Real-time sensor acquisition

The robot uses an onboard accelerometer (such as the MPU6050) to generate a coarse estimate of its position through numerical integration. This estimate is not intended to replace proper odometry but serves to demonstrate the use of inertial sensors in a real-time context.