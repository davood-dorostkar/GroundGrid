# Data merged from all sensors

time stamp

bool is_all_valid # Shortcut, true if all *_valid fields are true in this message

# Combined lidar point cloud
sensor_msgs/PointCloud2 combined_cloud # X, Y, Z, Intensity, R, G, B, Lidar id
bool combined_cloud_valid # True if combined_cloud contains all 4 invz lidars
bool cloud_front_left_valid # True if combined_cloud contains invz front left lidar
bool cloud_front_center_valid # True if combined_cloud contains invz front center lidar
bool cloud_front_right_valid # True if combined_cloud contains invz front right lidar
bool cloud_rear_center_valid # True if combined_cloud contains invz rear center lidar

# Camera images
sensor_msgs/CompressedImage cam_front_left
sensor_msgs/CompressedImage cam_front_center
sensor_msgs/CompressedImage cam_front_right
sensor_msgs/CompressedImage cam_rear_left
sensor_msgs/CompressedImage cam_rear_center
sensor_msgs/CompressedImage cam_rear_right
bool cam_front_left_valid
bool cam_front_center_valid
bool cam_front_right_valid
bool cam_rear_left_valid
bool cam_rear_center_valid
bool cam_rear_right_valid

# GPS data
nav_msgs/Odometry gps_odom
sensor_msgs/Imu gps_imu
bool gps_odom_valid
bool gps_imu_valid

# Vehicle (base_link) pose
geometry_msgs/PoseStamped vehicle_pose
bool vehicle_pose_valid
