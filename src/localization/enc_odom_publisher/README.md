# enc_odom_publisher
Uses the midpoint method to produce odometry. Publishes an Odometry message and broadcasts the odom -> base_link TF transform.

## Subscribed Topics
- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`) - Simulated encoder velocity

## Published Topics
- `odom` (`nav_msgs/Odometry`) - Robot odometry

## TF Broadcasts
- `odom -> base_link` - Derived from `odom_frame_id` and `base_frame_id` config
