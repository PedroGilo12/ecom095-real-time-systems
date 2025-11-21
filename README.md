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

# Task Specifications

## Task T1: Accelerometer Polling
- Type: Periodic
- Period (P): 10ms
- Cost (C): 1ms
- Deadline (D): 10ms (hard)

## Task T2: Odometry Calculation
- Type: Periodic
- Period (P): 10ms
- Cost (C): ?
- Deadline (D): 1ms (hard)

## Task T4: Finite State Machine for Bug1 Algorithm
- Type: Periodic
- Period (P): 20ms
- Cost (C): ?
- Deadline (D): 20ms (hard)

## Task T5: Bluetooth Goal Position Updater
- Type: Sporadic
- Min Interval (min/P): 1000ms
- Cost (C): ?
- Deadline (D): 1000ms (soft)

## Task T6: Bumpers Interrupt Handle
- Type: Sporadic
- Min Interval (min/P): 1000ms
- Cost (C): ?
- Deadline (D): 1000ms (soft)

# Finite State Machine for Bug 1 Algorithm (T4) description

```mermaid
flowchart LR
    A("Waiting<br>Goal")
    M("Motion<br>to Goal")
    B("Boundary<br>Following")
    D{"Bumper<br>active?"}

    A -->|GOAL_RECEIVED| D
    D -->|yes| B
    D -->|no| M
    B -->|M_LINE_FREE| M
    M -->|BUMPER_INTERRUPT| B
```