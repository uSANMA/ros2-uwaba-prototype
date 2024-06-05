[![Static Badge](https://img.shields.io/badge/version-v0.5.53-blue)](https://github.com/uSANMA/ros2-uwaba-prototype)
# uWABA NAVIGATION AND CONTROLLER SYSTEMS
This is the workspace related to uWABA's robot prototype ROS 2, Nav2 and ros2_control in Python[^1] systems. Here will be displayed every file and package related to its development.

## uSANMA SYSTEM'S FLOWCHART
<p align="center">
<img src="https://github.com/uSANMA/ros2-uwaba-prototype/blob/alpha/docs/uSANMA%20Stack%20English.png" width="500" align="center">
</p>

## PROJECT NOTES
### List of implementations
- [x] Add [URDF](https://github.com/uSANMA/ros2-uwaba-prototype/blob/alpha/packages/src/uwaba_prototype_description/urdf/uwaba_prototype.urdf) file from uwaba's model
- [x] Add [meshes](https://github.com/uSANMA/ros2-uwaba-prototype/tree/alpha/packages/src/uwaba_prototype_description/meshes/dae) file from uwaba's model
- Config directory implementations:
    - [ ] Implement Extended Kalman Filter file ([ekf.yaml](https://github.com/uSANMA/ros2-uwaba-prototype/blob/alpha/packages/src/uwaba_prototype_description/config/ekf.yaml))
    - [ ] Implement Navigation 2 parameters file ([nav2_params.yaml](https://github.com/uSANMA/ros2-uwaba-prototype/blob/alpha/packages/src/uwaba_prototype_description/config/nav2_params.yaml))[^2]
- Launch file implementations:
    - [x] Robot State Publisher node
    - [x] Joint State Publisher node
    - [x] RViz2 node
    - [x] Robot Stack node
    - [ ] Robot Localization node
    - [ ] `nav2_bringup`[^2] node
    

[^1]: We did a custom ros2_control in Python for simplicity of code.
[^2]: The `nav2_params.yaml` file will be launched within `nav2_bringup` tool.

<!-- Below are some examples on git's README text formatting -->
<!-- ### ODOMETRY
- `robot_localization` package can be exchanged by a [`tf2_broadcaster`](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html) if we don't want to fuse the odometry information provided the `IMU` and the `wheel encoders`.
- `robot_localization` can be used to publish the `odom -> base_link` transform to Nav2 stack that is already smoothed.
    - to summarize, the msgs that belongs to IMU (`sensor_msgs/Imu`) and wheel encoders (that need to be used by `ros2_control` so that it can export the `geometry_msgs/Twist` into `nav_msgs/Odometry`) will be published, respectively, to `/imu`[^1] and `/odom`[^2] topics that will be subscribed by the `robot_localization` node so that the final `odom` frame can be outputted.
    - multiple odometry's sensor sources can be fused by the `robot_localization` package's `ekf_node` (or `ukf_node`) that will then publish to the `odometry/filtered` and `accel/filtered` topics if enabled.
        - finally the `odom -> base_link` will be published to the `/tf` topic by the `robot_localization` once the setup has been managed.

[^1]: To configure the imu we need to specify in the YAML file's imu_config matrix which parameters should be used. As an example, if we only want to use roll, pitch and yaw from the imus configuration, we need to set the imu_config matrix as follows:imu0_config: [false, false, false, true,  true,  true, false, false, false, false, false, false, false, false, false]. This matrix sequence is set as follows -> x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az.
[^2]: The configuration related to the odom matrix is the same as stated in the IMU's.  -->
