# ROS 2 MuJoCo Simulation for Franka Robotics Panda (FER) 

This repository provides a high-fidelity physics simulation for the FER robot using **MuJoCo** and **ROS 2 Jazzy**. It utilizes the `mujoco_ros2_control` framework.

## Key Features
* **Modular URDF Wrapper**: Uses a top-level Xacro wrapper to inject MuJoCo physics without modifying upstream Franka packages.
* **Dockerized Workflow**: Consistent development environment with all dependencies (MuJoCo, KDL, ros2_control) pre-configured.

---

## Prerequisites

Ensure you have the following installed on your host machine:
* **Docker** (Latest version)
* **vcstool**: To manage repository dependencies.
  ```bash
  pip install vcstool
  ```

## Installation & Setup
1. Clone the Repository
```bash
git clone https://github.com/GKnerd/mujoco_sim_bringup.git
cd mujoco_sim_bringup
```

2. Import Dependencies
This project relies on external libraries. Use vcs to pull them into your workspace:

```bash
vcs import src < fer_ros2_mujoco.repos
```

3. Build the Docker Image
The build script handles the environment setup and compilation of the ROS 2 workspace:

```bash
./.docker/build_image.sh
```

## Running the Simulation
1. Launch the Container
Start the environment:

```bash
./.docker/run_image.sh
```

2. Bringup the Robot
Inside the container terminal, run the main launch file. This will open the MuJoCo simulator and activate the effort_controller for all 7 arm joints:

```bash
ros2 launch franka_mujoco_sim_bringup er_mujoco_ros2_control.launch.py
```
Note: The other launch files are purely experimental and will be removed.

## Project Structure
- urdf/: Contains the fer_mujoco.urdf.xacro wrapper. 
- config/: ROS 2 controller configurations (joint_state_broadcaster, effort_controller).
- scenes/: MuJoCo XML world definitions (floor, lighting).
- launch/: Contains the launch files for the bringup of the simulation.



## ⚠️ Contribution Guidelines
This repository is currently Work in Progress.

No direct commits to main: All development must happen in feature branches.

Branching: Please create a new branch for your specific task:

```bash
git checkout -b feature/your-feature-name
```
Ensure you run a clean `colcon build` before submitting a Pull Request.


