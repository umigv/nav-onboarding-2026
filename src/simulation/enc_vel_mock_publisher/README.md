# enc_vel_mock_publisher
Publishes a simulated encoder velocity (`enc_vel`) that drives a repeating square trajectory: four straight sides alternated with four 90-degree turns. A small constant drift bias plus gaussian noise is added to each velocity component to approximate real encoder error. A correct odometry node should trace a mostly closed square with a slight, consistent offset over time. 

## Published Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity
