# bringup
Launch files and shared configuration for the nav onboarding project.

## Configurations
### Frames
`config/frames.yaml` is the single source of truth for frame names (`base_frame`, `odom_frame`). Launch files load it via `bringup.launch_utils.load_frames()` instead of hardcoding frame name strings.

### RViz
`rviz/onboarding.rviz` is a saved RViz config with the Odometry, TF, and RobotModel displays already set up, fixed frame set to `odom`. Launch RViz plain alongside the other nodes, then load it via **File → Open Config**, and browse to `src/bringup/rviz/onboarding.rviz` in the repo:
```
rviz2
```

---

## core.launch.py
Launches `robot_state_publisher` (from `maverick_description/urdf/maverick.xacro`) and `joint_state_publisher` to produce a model of the robot to be visualized.
```
ros2 launch bringup core.launch.py
```

---

## localization.launch.py
Empty template. Add your odometry node here.
```
ros2 launch bringup localization.launch.py
```

---

## simulation.launch.py
Launches `enc_vel_mock_publisher` (a live square-trajectory encoder velocity publisher with a small amount of drift). Run alongside `core.launch.py` and `localization.launch.py` (see above), and RViz (see above), as three separate launches.
```
ros2 launch bringup simulation.launch.py
```

### Published Topics
- `enc_vel/raw` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity driving a square trajectory
