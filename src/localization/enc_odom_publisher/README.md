# enc_odom_publisher

Integrates encoder velocity into a planar pose and publishes odometry and TF.
## Subscribed Topics

- `enc_vel` (`geometry_msgs/TwistWithCovarianceStamped`): Encoder velocity. The launch file remaps this to `enc_vel/raw`.

## Published Topics
- `odom` (`nav_msgs/Odometry`): Estimated robot odometry.

## Services
- `reset_odom` (`std_srvs/Trigger`): Resets the estimated pose.

## TF Broadcasts
- `odom` to `base_link`

## Subscribed Topics
- `topic` (`pkg/Msg`) - Description

## Published Topics
- `topic` (`pkg/Msg`) - Description
