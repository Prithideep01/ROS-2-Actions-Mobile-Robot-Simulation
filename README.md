# 🤖 ROS 2 Actions: Mobile Robot Simulation

This repository contains a robust implementation of a **ROS 2 Action Server and Client** designed to simulate and control a mobile robot's linear motion. It demonstrates advanced action concepts including asynchronous goal execution, real-time feedback, goal preemption and user-triggered cancellation.

![ROS 2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white) ![Python](https://img.shields.io/badge/Language-Python%203-3776AB?logo=python&logoColor=white)

---

## 🚀 Key Features

* **Custom Action Interface:** Uses a dedicated `.action` file defining Goal (Position/Velocity), Feedback (Current Position), and Result (Final Status).
* **Overshoot Prevention:** Intelligent logic in the server ensures the robot snaps precisely to the target coordinate without overshooting, even if the velocity step exceeds the remaining distance.
* **Goal Preemption Policy:** Implements a "Newest Goal Wins" policy. If a new goal is received while the robot is moving, the current goal is automatically aborted/preempted to prioritize the new command.
* **Graceful Cancellation:** Both Server and Client handle cancellation requests safely. The client includes a subscriber bridge allowing users to trigger stops via a standard ROS topic (`/cancel_move`).
* **Multi-Threaded Execution:** Utilizes `MultiThreadedExecutor` and `ReentrantCallbackGroup` to ensure responsive handling of callbacks (cancel, feedback, and goal processing) simultaneously.


## 🛠️ System Architecture

### 1. Action Interface (`MoveRobot.action`)
* **Goal:** `int64 goal_position`, `int64 desired_velocity`
* **Result:** `int64 final_position`, `string message`
* **Feedback:** `int64 current_position`

### 2. The Action Server
* **Initial State:** Robot starts at **50 meters**.
* **Logic:** Simulates movement at the requested velocity (m/s).
* **Safety:** Validates goals (rejects negative inputs) and handles thread locking for safe preemption.

### 3. The Action Client
* **Functionality:** Sends goals to the server and processes asynchronous feedback.
* **Cancel Bridge:** Subscribes to `/cancel_move`. Publishing an `Empty` message to this topic triggers a cancellation request to the server.

---
