# F1Tenth Project: Safety, Control, and Planning

This repository contains the implementation of key modules for the F1Tenth autonomous car project, focusing on safety, control, and planning. It also includes a simulation environment written in **C++/C** for validating and fine-tuning algorithms before deployment on the real car.

---

## Project Features

### 1. Automatic Emergency Braking (AEB)
- **Language**: Python
- **Description**: Implements a Time-to-Collision (TTC)-based safety mechanism to ensure the car halts safely at varying speeds and distances.
- **Key Highlights**:
  - Dynamically tuned `TTC_threshold` for precise braking.
  - Validated through multiple real-world tests, ensuring reliable collision prevention.

### 2. Wall Following with PD Controller
- **Language**: Python
- **Description**: Utilizes a Proportional-Derivative (PD) Controller to enable wall-following behavior, maintaining a consistent distance from the wall on a track.
- **Key Highlights**:
  - Optimized `Kp` and `Kd` parameters for smooth navigation.
  - Achieves stable multi-lap performance without collisions.

### 3. Follow-the-Gap Algorithm
- **Language**: Python
- **Description**: Implements a reactive obstacle avoidance algorithm by identifying the largest gap in the environment and dynamically adjusting the car's path.
- **Key Highlights**:
  - Adaptive navigation for dynamic obstacle scenarios.
  - Seamless completion of multiple laps with obstacle avoidance.

---

## Simulation Environment (C++/C)
The `simulation` folder contains **C++/C** implementations for various labs to validate algorithms before real-world deployment. These labs are designed to progressively build your understanding of ROS and autonomous driving techniques.

### Labs Overview
1. **Lab 0: Basic ROS**  
   Introduction to the ROS framework, essential commands, and setting up the environment.
   
2. **Lab 1: Basic ROS and F1Tenth Simulator**  
   Setting up and running the F1Tenth simulator to familiarize with the simulation environment.

3. **Lab 2: Automatic Emergency Braking in Simulation**  
   Implementation of the AEB algorithm using Time-to-Collision (TTC) to stop the car safely.

4. **Lab 3: Control - Wall Following with PID Controller in Simulation**  
   Deploying a PID Controller to enable wall-following behavior with parameter tuning (`Kp`, `Ki`, `Kd`).

5. **Lab 4: Path Planning - Reactive Methods: Follow the Gap in Simulation**  
   Developing a reactive obstacle avoidance algorithm using the Follow-the-Gap method.

### Running the Simulation
1. Navigate to the specific lab directory:
   ```bash
   cd simulation/labX
   ```
2. Build the code:
   ```bash
   catkin_make
   ```
3. Run the simulation node:
   ```bash
   rosrun <package_name> <labX_node>
   ```

---

## Repository Structure
```
.
├── src/                  # Source code for all main modules
│   ├── aeb.py            # Automatic Emergency Braking
│   ├── wall_following.py # PD Controller for Wall Following
│   ├── gap_following.py  # Follow-the-Gap Algorithm
├── simulation/           # C++/C simulation code for initial testing
│   ├── lab0/             # Basic ROS
│   ├── lab1/             # Basic ROS and F1Tenth Simulator
│   ├── lab2/             # AEB in Simulation
│   ├── lab3/             # Wall Following in Simulation
│   ├── lab4/             # Follow-the-Gap in Simulation
└── README.md             # Project documentation
```

---

## Requirements
- **Languages**: Python, C, C++
- **Framework**: ROS (Robot Operating System)
- **Hardware**: F1Tenth Car, Jetson Xavier NX
- **Tools**: 
  - `rqt_plot` for visualizing test data.
  - ROS Bag for recording data during experiments.

---

## Setup and Usage
### Clone the Repository
```bash
git clone https://github.com/bryaz7/F1Tenth-Car.git
cd F1Tenth-Car
```

### Running Python Nodes
1. **Automatic Emergency Braking (AEB)**:
   ```bash
   rosrun <package_name> aeb.py
   ```
2. **Wall Following with PD Controller**:
   ```bash
   rosrun <package_name> wall_following.py
   ```
3. **Follow-the-Gap Algorithm**:
   ```bash
   rosrun <package_name> gap_following.py
   ```

### Running C++/C Simulation Code
1. Navigate to the specific lab directory:
   ```bash
   cd simulation/labX
   ```
2. Build the code:
   ```bash
   catkin_make
   ```
3. Run the lab node:
   ```bash
   rosrun <package_name> <labX_node>
   ```

---

## Demonstration Videos
- **Automatic Emergency Braking (AEB)**: [https://www.youtube.com/watch?v=pSp69bPrjUI](#)
- **Wall Following**: [https://www.youtube.com/watch?v=UAqGMAAkr50](#)
- **Follow-the-Gap Algorithm**: [https://www.youtube.com/shorts/jWRoxzqvO_U](#)

---

## Documentation and Reports
Comprehensive details about the implementation, testing, and results can be found in the project report. The report includes:
- Algorithm explanations.
- Parameter tuning methods.
- Challenges faced and solutions implemented.
- Data visualizations and performance metrics.

---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.