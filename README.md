*Note: The prebuilt XML examples are located inside the container at `ros2_ws/build/mujoco_vendor/mujoco_prebuilt_3.4.0/`.*


# Franka Description URDF Modifications

- Added the relevant Mujoco Tags inside the URDF so that it can be converted to a parsable .mjcf file.
- To convert the URDF to MJCF we use the script provided by PAL Robotics. 
- The way Mujoco works is, it expects a world file with models loaded in. So the URDF needs to be passed to the transition script with a `--scene`argument so that we 
can build the simulation with the robot inside the Mujoco World. 


- Mujoco builds the simulation upfront, different from Gazebo you cannot spawn objects at runtime. The scene has to be preloaded and ready, before the simulation starts. The mjModel is locked and immutable.
