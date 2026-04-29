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
│── carla_sync_UGV_UAV.py     # Main simulation runner
│── config.py                 # Simulation configuration and parameters
│── data_logger.py            # Logs simulation and sensor data
│── ed2_avoid.py              # Obstacle avoidance implementation
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

### 1. Start CARLA Server

```bash
CarlaUE4.exe
```

### 2. Run Simulation

```bash
python carla_sync_UGV_UAV.py
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
