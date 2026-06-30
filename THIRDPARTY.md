# Third-Party Software Notices

This repository integrates and depends on several open-source software
components. We gratefully acknowledge the developers and contributors of
the following projects. Each component is provided under the license
indicated; the full license texts are available at the linked sources.

Author's own packages under `ros2_ws/src/` (`fer_skills`,
`franka_mujoco_sim_bringup`, `speed_and_separation_monitoring`,
`fer_moveit_config`) carry their own `LICENSE` and, where relevant,
`NOTICE.md` files and are not listed here.

-------------------------------------------------------------------------
1. ROS 2 (Robot Operating System), `ros:jazzy-ros-base` base image
-------------------------------------------------------------------------
Copyright (c) Open Source Robotics Foundation (OSRF) / Open Robotics
and contributors.
License: Apache License 2.0
Source: https://github.com/ros2
Base image: https://hub.docker.com/_/ros (`ros:jazzy-ros-base`)

ROS 2 is an open-source middleware suite for robot software development.
This project targets the Jazzy Jalisco distribution and uses the
official OSRF-maintained Docker base image.

-------------------------------------------------------------------------
2. MuJoCo (Multi-Joint dynamics with Contact)
-------------------------------------------------------------------------
Copyright (c) DeepMind Technologies Limited.
License: Apache License 2.0
Source: https://github.com/google-deepmind/mujoco

MuJoCo is an open-source physics engine used as the simulation back end
for this project.

-------------------------------------------------------------------------
3. mujoco_vendor
-------------------------------------------------------------------------
Copyright (c) PAL Robotics.
License: Apache License 2.0
Source: https://github.com/pal-robotics/mujoco_vendor

CMake vendor package that integrates the MuJoCo binary into the ROS 2
build ecosystem.

-------------------------------------------------------------------------
4. mujoco_ros2_control
-------------------------------------------------------------------------
Copyright (c) ros-controls working group and contributors.
License: Apache License 2.0
Source: https://github.com/ros-controls/mujoco_ros2_control

Hardware interface that bridges MuJoCo with the `ros2_control` framework.

-------------------------------------------------------------------------
5. franka_description
-------------------------------------------------------------------------
Copyright (c) Franka Robotics GmbH.
License: Apache License 2.0
Source: https://github.com/frankarobotics/franka_description

Official URDF descriptions, meshes, and model files for the Franka
Emika Robot.

-------------------------------------------------------------------------
6. BehaviorTree.ROS2
-------------------------------------------------------------------------
Copyright (c) Davide Faconti and contributors.
License: see upstream — the repository's top-level `LICENSE` file is
Apache License 2.0, while individual `package.xml` files declare MIT.
Both notices are preserved in the vendored source tree under
`ros2_ws/src/BehaviorTree.ROS2/`.
Source: https://github.com/BehaviorTree/BehaviorTree.ROS2

ROS 2 bindings and node/action server templates for BehaviorTree.CPP.

-------------------------------------------------------------------------
7. Docker tooling — `docker/Dockerfile`, `docker/build_image.sh`,
   `docker/run_container.sh`
-------------------------------------------------------------------------
Copyright (c) 2025 Proximity Robotics & Automation GmbH.
Modifications copyright (c) 2026 Georgios Katranis.
License: Apache License 2.0
Source: <internal — no public repository>

The Dockerfile layering pattern and the `build_image.sh` /
`run_container.sh` shell scripts are derived from internal tooling
developed at Proximity Robotics & Automation GmbH. The Dockerfile has
been adapted for the FER + MuJoCo dependency set, CycloneDDS
configuration, and the colcon build stage. Each file retains the
original copyright notice and carries a "modifications by" note per
Apache 2.0 § 4(b).

-------------------------------------------------------------------------

Unless otherwise noted above, the listed software is licensed under the
Apache License, Version 2.0 (the "License"); you may not use these files
except in compliance with the License. You may obtain a copy of the
License at:

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
implied. See the License for the specific language governing permissions
and limitations under the License.
