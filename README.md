# Vektor-E: 1/10th Scale Autonomous Vehicle Control System

## 🎯 Overview

This repository contains the C++ control system for a 1/10th scale autonomous vehicle. It processes LiDAR data for navigation and integrates Reinforcement Learning models (trained with Tianshou/Python in Webots) for autonomous driving. The system is designed for Jetson Nano embedded platforms.

Architecture diagrams:
*   `ArchitectureRS.png`
*   `ArchitectureRS_Txt.png`

## 🔗 Getting Started (Git Submodules)

This project uses Git submodules. To clone:
```bash
git clone --recurse-submodules <this_repository_url>
```
If already cloned, or to update submodules:
```bash
git submodule update --init --recursive
```
Submodules include `RL` (Reinforcement Learning components), `JetsonGPIO`, and `LiDAR_Project` (LiDAR tools).

## 🛠️ Prerequisites

*   C++ Compiler (g++ supporting C++20).
*   Python (for the `RL` submodule).
*   Libraries: `libpthread`, `libtbb-dev`.
*   Slamtec RPLIDAR SDK (`libsl_lidar_sdk.a` in `LiDAR_Project/lib/`).
*   `JetsonGPIO` library (via submodule).
*   **Hardware:** 1/10th scale vehicle, LiDAR, Jetson Nano (or similar).

## ⚙️ Compilation

The main application `HammerTime` is compiled using `compile.sh`:
```bash
./compile.sh
```
This creates the `HammerTime` executable, linking `main.cpp`, LiDAR interface, `Controller_Txt.cpp`, `Uart_Txt.cpp`, and `VektorSystem.cpp`. It uses file-oriented `_Txt` components by default.

## ▶️ Execution

Run the application using `run.sh`:
```bash
./run.sh
```
This script sets permissions for the LiDAR's serial port (`/dev/ttyUSB0`) and launches `./HammerTime`.
The LiDAR port and baud rate (`256000`) are hardcoded in `main.cpp`.

```
