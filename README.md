# Dynamic Median Traffic Management System

[![CARLA](https://img.shields.io/badge/CARLA-0.9.15-blue.svg)](https://carla.org/)
[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

An intelligent traffic management system that dynamically adjusts highway lane configurations using AI-powered congestion detection and a movable median barrier. This project addresses the **Engineering Grand Challenge** of urban infrastructure optimization and traffic congestion mitigation.

![CARLA Simulation](images/simulation_screenshot.png)
*Real-time traffic simulation in CARLA showing the dynamic median traffic management system*

## 🎯 Overview

Traditional highway infrastructure suffers from directional traffic imbalances—morning rush hours congest one direction while the opposite remains empty, and vice versa in the evening. This system solves that problem by:

---

## 🚀 Features

### Core Functionality
- **Intelligent Congestion Detection**: Monitors vehicle speeds and counts in real-time
- **Adaptive Lane Allocation**: Shifts median barrier to create 4 lanes in congested direction
- **Computer Vision**: YOLOv8-based vehicle detection and tracking
- **BPR Function Analysis**: Bureau of Public Roads trip time calculations
- **Real-time Dashboard**: Web-based monitoring and control interface

### Technical Capabilities
- Simulates 6-lane highways (3 lanes each direction)
- Handles 100+ vehicles simultaneously
- Dynamic median movement with collision avoidance
- Traffic flow metrics and performance analysis
- Export data to JSON/CSV for further analysis

---

## 📋 Requirements

### Software Dependencies
```
Python 3.8+
CARLA Simulator 0.9.15
```

### Python Packages
```
carla==0.9.15
pygame==2.5.2
flask==3.0.0
numpy (optional, for advanced analysis)
```

---

## 🔧 Installation

### 1. Install CARLA Simulator
Download CARLA 0.9.15 from [official website](https://github.com/carla-simulator/carla/releases/tag/0.9.15)

```bash
# Extract CARLA
# Windows: Extract to C:\CARLA
# Linux: Extract to ~/CARLA
```

### 2. Clone Repository
```bash
git clone https://github.com/hitX/dynamic-median-traffic.git
cd dynamic-median-traffic
```

### 3. Set Up Python Environment
```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Linux/Mac:
source venv/bin/activate

# Install dependencies
pip install carla==0.9.15 pygame==2.5.2 flask==3.0.0
```

---

## 🎮 Quick Start

### 1. Start CARLA Simulator
```bash
# Windows
cd C:\CARLA
.\CarlaUE4.exe

# Linux
cd ~/CARLA
./CarlaUE4.sh
```

### 2. Run Simulation
```bash
# In project directory
python test_carla.py
```

### 3. Launch Dashboard (Optional)
```bash
# In a separate terminal
python dashboard_server.py
```
Then open `http://localhost:5000` in your browser.

---

## 📊 Usage

### Basic Simulation
The simulation runs automatically and will:
1. Spawn 100+ vehicles on a 6-lane highway
2. Monitor traffic conditions in real-time
3. Detect congestion (>15 slow vehicles)
4. Shift median to create 4-2 lane configuration
5. Return to 3-3 when congestion clears

### Dashboard Controls
- **Shift Median**: Manually trigger lane configuration changes
- **Spawn Vehicles**: Add more traffic to test scenarios
- **Speed Control**: Adjust traffic flow speed
- **Weather**: Change environmental conditions
- **Create Congestion**: Simulate rush hour traffic

### Keyboard Controls
- `Ctrl+C`: Stop simulation and save metrics
- `ESC`: Emergency stop

---

## 📈 Performance Metrics

The system calculates and tracks:

### 1. Trip Time Improvement
Uses BPR (Bureau of Public Roads) function:
```
T = T₀ × [1 + 0.15 × (V/C)⁴]

Where:
- T₀ = Free-flow time (10-60 minutes, random)
- V = Traffic volume (220-250 vehicles/hour)
- C = Capacity (200 baseline, 267 improved)
```

## 📁 Project Structure

```
median_3d/
│
├── test_carla.py              # Main simulation script
├── dashboard_server.py         # Web dashboard backend
├── templates/
│   └── metrics_dashboard.html  # Dashboard UI
│
├── ANALYSIS.md                 # Performance analysis
├── METHODS.md                  # Methodology documentation
├── METRICS_EQUATIONS.md        # Formula reference
├── QUICK_START.md              # User guide
│
├── simulation_results.json     # Historical test data
└── simulation_state.json       # Real-time state
```

---

## 🔬 Technical Details

### BPR Function Implementation
The Bureau of Public Roads function calculates trip time based on traffic volume and capacity:

**Baseline (3-3 lanes):**
- Capacity: 200 veh/h per direction

**Improved (4-2 lanes):**
- Capacity: 267 veh/h (forward)


---

## 📖 Documentation

Detailed documentation available:

- **[ANALYSIS.md](ANALYSIS.md)** - Complete performance analysis with graphs and tables
- **[METHODS.md](METHODS.md)** - Materials, construction, and test methodology
- **[METRICS_EQUATIONS.md](METRICS_EQUATIONS.md)** - All formulas and calculations
- **[QUICK_START.md](QUICK_START.md)** - Step-by-step user guide

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👥 Authors

**Mohamed Eltahawy**

---

## 🙏 Acknowledgments

- **CARLA Team** - Open-source autonomous driving simulator
- **National Academy of Engineering** - Grand Challenges inspiration
- **Texas A&M Transportation Institute** - Traffic congestion research data
- **Bureau of Public Roads** - BPR function methodology

---


## 🔗 Links

- [CARLA Simulator](https://carla.org/)

---

**⭐ Star this repository if you find it helpful!**
