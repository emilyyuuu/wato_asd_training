# Autonomous Robot Navigation System

An autonomous robot navigation system built with **ROS2** that allows a simulated differential-drive robot to navigate from point A to point B while avoiding obstacles. The system processes **LiDAR sensor data** to build an occupancy grid map, plans a path using **A\*** search, and follows the path using **Pure Pursuit control**.

This project was completed as part of the **WATonomous Autonomous Software training**, focusing on robotics software architecture and autonomous navigation.

## Features

**LiDAR-Based Mapping**
- Converts LiDAR scan data into a 2D occupancy grid representing obstacles in the environment.

**Autonomous Path Planning**
- Uses the **A\*** algorithm to compute an optimal path from the robot’s current position to a target location.

**Global Map Construction**
- Aggregates costmaps over time to maintain a consistent map of the environment.

**Trajectory Tracking**
- Implements **Pure Pursuit control** to follow the planned path and generate velocity commands.

**ROS2 Modular Architecture**
- Built with multiple ROS2 nodes that communicate through topics for perception, planning, and control.

## Tech Stack

- **C++**
- **ROS2**
- **Docker**
- **Foxglove Studio**

## Demo

[![Watch the demo](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=EhbpIKTd3rA)

## Installation

### Clone the repository
```bash
git clone https://github.com/WATonomous/wato_asd_training.git
cd wato_asd_training
