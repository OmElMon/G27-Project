# UAV–UGV Coordination System (CARLA)

## Project Overview
This project focuses on building a simulation system for coordination between UAVs (Unmanned Aerial Vehicles) and UGVs (Unmanned Ground Vehicles) using the **CARLA simulator**.

The system enables autonomous navigation, obstacle avoidance, sensor integration, and communication between agents in a controlled virtual environment.

## Objectives
- Simulate UAV and UGV coordination in CARLA
- Implement obstacle avoidance using LiDAR sensors
- Enable communication via a message broker system
- Collect and log simulation data for analysis
- Provide a modular architecture for team development

## Key Features
- UGV and UAV control systems
- Sensor integration: LiDAR, camera, etc.
- Obstacle avoidance logic
- Message broker for inter-agent communication
- Data logging system
- Configurable simulation parameters


## Project Structure

```text
G27-Project/
│── reference/                # Semester 1 demo/reference code
│── .gitignore
│
│── carla_sync_UGV_UAV.py     
│── config.py                 # Simulation configuration and parameters
│── coordination_platform.py  # Implements leader-follower logic
│── data_logger.py            # Logs simulation and sensor data
│── ed2_avoid.py              # Obstacle avoidance implementation
│── gui_console.py            # Implements the GUI
│── gui_main.py               # Main simulation runner, displays GUI
│── main.py                   # Headless simulation runner, no GUI, uses Carla display
│── message_broker.py         # Communication between UAV and UGV
│── sensor_manager.py         # Sensor setup and handling
│── uav_controller.py         # UAV movement and logic
│── ugv_controller.py         # UGV movement and obstacle avoidance
```

## Technologies Used
- Python
- CARLA Simulator
- CARLA Python API
- LiDAR Sensors
- Git and GitHub

---

## How to Run

## Prerequisites

- Windows 10 or 11
- [CARLA 0.9.16](https://github.com/carla-simulator/carla/releases) installed and extracted
- Python 3.12 (install from [python.org](https://www.python.org/downloads/)

## Setup

Open a terminal in the project root (the folder containing this README) and run
the steps below.

### 1. Create and activate a virtual environment

```powershell
py -3.12 -m venv venv
```

Activate it. The activation command depends on your shell:

- **PowerShell**:
  ```powershell
  .\venv\Scripts\Activate.ps1
  ```
  If PowerShell blocks the script with an execution policy error, run this once
  in an elevated PowerShell:
  ```powershell
  Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
  ```

- **cmd.exe**:
  ```cmd
  venv\Scripts\activate.bat
  ```

- **Git Bash**:
  ```bash
  source venv/Scripts/activate
  ```

### 2. Install the CARLA Python API wheel

The `carla==0.9.16` line in `requirements.txt` will pull from PyPI on most
machines, but if that fails (Windows wheels for some CARLA versions are not
published), install the wheel shipped with your CARLA installation directly:

```powershell
pip install "C:\path\to\CARLA_0.9.16\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"
```

Replace the path with wherever you extracted CARLA. 

### 3. Install the remaining dependencies

```powershell
pip install -r requirements.txt
```

## Running the Project

### 1. Start the CARLA server

In a separate terminal, launch the simulator:

```powershell
cd C:\path\to\CARLA_0.9.16
.\CarlaUE4.exe
```

Wait until the CARLA window finishes loading the default map.

### 2. Run the simulation

Two entry points are available. Both will prompt interactively for navigation
mode, follow distance, follow altitude, and camera target.

**GUI console (recommended)** — full operator console with chase camera, UAV
picture-in-picture, live state readout, and a clickable minimap that lets you
re-route the UGV at runtime:

```powershell
python main_gui.py
```

**Headless / terminal mode** — simpler runner with a small UAV camera popup:

```powershell
python main.py
```

Both accept optional CLI flags:

```powershell
python main_gui.py --host localhost --port 2000 --distance 25 --altitude 30
```

### 3. Stop the simulation

- Click the **Quit** button in the GUI console, or close the window
- Press `Ctrl+C` in the terminal
- Or wait for the UGV to reach its destination

CSV logs are written to `./logs/` after each run.

## Running Individual Subsystems

Each subsystem can be run standalone for testing. Make sure the CARLA server
is running first.

```powershell
python ugv_controller.py        # UGV-only test (autopilot or scripted path)
python uav_controller.py        # UAV-only test
python coordination_platform.py # Coordination logic with simulated messages
```
---

## Current Progress
- CARLA environment configured
- UAV and UGV controllers implemented
- LiDAR-based obstacle avoidance added
- Message broker for communication working
- Data logging system implemented

## Future Work
- Advanced UAV–UGV coordination strategies
- Real-time path planning algorithms
- Multi-agent scaling
- Improved sensor fusion
- Visualization/dashboard for analytics

## Team Members
- **Evan Frisone** – Team Leader, Computer Science
- **Omar Elharbili** – Computer Science
- **Sean Bowden** – Computer Science
- **Roberson Robert** – Team Member
- **Syeda Haque** – Computer Science

## Sponsor / Mentor
**Dr. Xiangnan Zhong**  
Email: xzhong@fau.edu

## Institution
Florida Atlantic University


## Notes
This project is part of a collaborative academic initiative focused on autonomous systems, robotics simulation, and intelligent transportation research.
