# ROS 2 MuJoCo Simulation for Franka Robotics Panda (FER)

This repository provides a high-fidelity physics simulation for the FER robot using **MuJoCo** and **ROS 2 Jazzy**. It uses the `mujoco_ros2_control` framework and ships a fully Dockerized workflow with CycloneDDS for low-latency inter-process communication.

## Key Features
* **Modular URDF Wrapper**: Uses a top-level Xacro wrapper to inject MuJoCo physics without modifying upstream Franka packages.
* **Dockerized Workflow**: Consistent development environment with all dependencies (MuJoCo, MoveIt, ros2_control, CycloneDDS) pre-configured.

---

## Prerequisites

Ensure the following are installed on your host machine:
* **Docker** (latest version)
* **vcstool**: To manage repository dependencies.
  ```bash
  pip install vcstool
  ```

## Installation & Setup

1. Clone the repository:
```bash
git clone https://github.com/GKnerd/fer_ros2_simulation.git fer_ros2_mjc_docker
cd fer_ros2_mjc_docker
```

2. Create the source directory and import dependencies:
```bash
mkdir -p ros2_ws/src
vcs import ros2_ws/src < fer_ros2_mjc.repos
```

3. Build the Docker image:
```bash
./docker/build_image.sh
```
This builds the image, compiles the full ROS 2 workspace inside the container, and copies the built workspace back to `ros2_ws/` on your host.

## Running the Simulation

0. Prelaunch setup:
**Crucial**
CycloneDDS requires a massive receive buffer to reliably handle large trajectory messages without dropping packets. You must configure your Host OS kernel to allow this, or the Docker container will crash on boot.
```bash
sudo sysctl -w net.core.rmem_max=2147483647
```
>Note 1: To make this persist across reboots, add `net.core.rmem_max=2147483647` to your `/etc/sysctl.conf file`.
>Note 2: You don´t have to specify 2GBs as here, this is use case specific. The value was chosen arbitratily in this case. 

1. Launch the container:
```bash
./docker/run_container.sh
```

2. Inside the container, run the main launch file:
```bash
ros2 launch franka_mujoco_sim_bringup fer_mujoco_ros2_control.launch.py
```
This opens the MuJoCo simulator and activates the effort controller for the arm and an effort controller for the hand.

If you wish to use the Mujoco simulator with MoveIt and an activated effort controller for the hand and arm you can use this launch file instead.

```bash
ros2 launch franka_mujoco_sim_bringup fer_mujoco_moveit.launch.py
```

## Repository Structure

```
fer_ros2_mjc_docker/
├── docker/
│   ├── Dockerfile              # Image definition (ROS 2 Jazzy + MoveIt + CycloneDDS)
│   ├── build_image.sh          # Builds the Docker image and extracts the built workspace
│   └── run_container.sh        # Runs the container with GPU/display/volume mounts
├── env/
│   └── cyclone_dds.xml         # CycloneDDS middleware profile
├── ros2_ws/
│   └── src/                    # Populated by vcs import (see Installation)
├── fer_ros2_mjc.repos          # VCS dependency manifest
├── DDS_Profiles.md             # Reference guide for CycloneDDS configuration and some additional experimental tips which are not on main
├── THIRDPARTY.md               # Third-party attribution notes
└── LICENSE
```

### Workspace packages (imported via `fer_ros2_mjc.repos`)
| Package | Description |
|---|---|
| `franka_description` | Official Franka Robotics URDF and meshes |
| `franka_mujoco_sim_bringup` | Launch files, URDF wrapper, and controller configs |
| `mujoco_ros2_control` | MuJoCo hardware interface for `ros2_control` |
| `mujoco_vendor` | CMake vendor package that integrates the MuJoCo binary |
| `fer_moveit_config` | MoveIt configuration for the FER robot |
| `fer_skills` | Reusable manipulation skills built on MoveIt |
| `speed_and_separation_monitoring` | ISO/TS 15066 speed-and-separation monitoring node |
| `BehaviorTree.ROS2` | BehaviorTree.CPP bindings for ROS 2 |

---

## Contribution Guidelines

This repository is currently a work in progress.

* **No direct commits to `main`**: All development must happen in feature branches.
* Create a new branch for your task:
  ```bash
  git checkout -b feature/your-feature-name
  ```
* Run a clean `colcon build` before submitting a Pull Request.
