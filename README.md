[![Static Badge](https://img.shields.io/badge/version-v0.25.0-green)](https://github.com/uSANMA/ros2-uwaba-prototype)
[![Version](https://img.shields.io/badge/dynamic/json?label=version&query=%24[0].name&url=https%3A%2F%2Fapi.github.com%2Frepos%2FuSANMA%2Fros2-uwaba-prototype%2Ftags)](https://github.com/uSANMA/ros2-uwaba-prototype)

# uWABA NAVIGATION AND CONTROLLER SYSTEMS
This is the workspace related to uWABA's robot prototype ROS 2, Nav2 and ros2_control in Python[^1] systems. Here will be displayed every file and package related to its development.

## uSANMA SYSTEM'S FLOWCHART
<p align="center">
<img src="https://github.com/uSANMA/ros2-uwaba-prototype/blob/beta/docs/uSANMA%20Stack%20English.png" width="800" align="center">
</p>

## REQUIRED PACKAGES
```bash
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
sudo apt install ros-humble-xacro -y
sudo apt install ros-humble-joint-state-publisher-gui -y
sudo apt install ros-humble-tf-transformations -y
sudo apt install ros-humble-robot-localization -y
sudo apt install ros-humble-urdf-tutorial -y
sudo pip3 install transforms3d
```

## BUILD PROJECT
```bash
<source_ros2_installation>
colcon build --packages-select uwaba_prototype_interfaces uwaba_prototype_description uwaba_prototype_diffbot_py
```

## Micro XRCE-DDS Agent Installation
```bash
<source_ros2_installation>
git clone -b ros2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd ros2-uwaba-prototype/packages/
colcon build --packages-select microxrcedds_agent
source install/local_setup.bash
```

## PROJECT NOTES
### List of implementations
- [x] Add [URDF](https://github.com/uSANMA/ros2-uwaba-prototype/blob/beta/packages/src/uwaba_prototype_description/urdf/uwaba_prototype.urdf.xacro) file from uwaba's model
- [x] Add [meshes](https://github.com/uSANMA/ros2-uwaba-prototype/tree/beta/packages/src/uwaba_prototype_description/meshes/dae) file from uwaba's model
- Config directory implementations:
    - [x] Implement Extended Kalman Filter file ([ekf.yaml](https://github.com/uSANMA/ros2-uwaba-prototype/blob/beta/packages/src/uwaba_prototype_diffbot_py/config/ekf.yaml))[^3]
    - [x] Implement Navigation 2 parameters file ([nav2_params.yaml](https://github.com/uSANMA/ros2-uwaba-prototype/blob/beta/packages/src/uwaba_prototype_diffbot_py/params/nav2_params.yaml))[^2]
- Launch file implementations:
    - [x] Robot State Publisher node
    - [x] Joint State Publisher node
    - [x] RViz2 node
    - [x] Robot Stack node
    - [x] Robot Localization node
    - [x] [`nav2_bringup`](https://github.com/uSANMA/ros2-uwaba-prototype/blob/beta/packages/src/uwaba_prototype_diffbot_py/launch/bringup_launch.py)[^2] node
    

[^1]: We did a custom ros2_control in Python for simplicity of code.
[^2]: The `nav2_params.yaml` file will be launched within `nav2_bringup` tool.
[^3]: All state estimation nodes track the 15-dimensional state of the vehicle: (X,Y,Z,roll,pitch,yaw,X˙,Y˙,Z˙,roll˙,pitch˙,yaw˙,X¨,Y¨,Z¨)

<!-- Below are some examples on git's README text formatting -->
<!-- ### ODOMETRY
- `robot_localization` package can be exchanged by a [`tf2_broadcaster`](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html) if we don't want to fuse the odometry information provided the `IMU` and the `wheel encoders`.
- `robot_localization` can be used to publish the `odom -> base_link` transform to Nav2 stack that is already smoothed.
    - to summarize, the msgs that belongs to IMU (`sensor_msgs/Imu`) and wheel encoders (that need to be used by `ros2_control` so that it can export the `geometry_msgs/Twist` into `nav_msgs/Odometry`) will be published, respectively, to `/imu`[^1] and `/odom`[^2] topics that will be subscribed by the `robot_localization` node so that the final `odom` frame can be outputted.
    - multiple odometry's sensor sources can be fused by the `robot_localization` package's `ekf_node` (or `ukf_node`) that will then publish to the `odometry/filtered` and `accel/filtered` topics if enabled.
        - finally the `odom -> base_link` will be published to the `/tf` topic by the `robot_localization` once the setup has been managed.

[^1]: To configure the imu we need to specify in the YAML file's imu_config matrix which parameters should be used. As an example, if we only want to use roll, pitch and yaw from the imus configuration, we need to set the imu_config matrix as follows:imu0_config: [false, false, false, true,  true,  true, false, false, false, false, false, false, false, false, false]. This matrix sequence is set as follows -> x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az.
[^2]: The configuration related to the odom matrix is the same as stated in the IMU's.  -->
