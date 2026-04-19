# IMU state (16-dim)
std_msgs/Header header

float64 pos_x          # position x (m)
float64 pos_y          # position y (m)
float64 pos_z          # position z (m)
float64 vel_x          # velocity x (m/s)
float64 vel_y          # velocity y (m/s)
float64 vel_z          # velocity z (m/s)
float64 bias_gyro_x    # gyroscope bias x (rad/s)
float64 bias_gyro_y    # gyroscope bias y (rad/s)
float64 bias_gyro_z    # gyroscope bias z (rad/s)
float64 bias_accel_x   # accelerometer bias x (m/s^2)
float64 bias_accel_y   # accelerometer bias y (m/s^2)
float64 bias_accel_z   # accelerometer bias z (m/s^2)
float64 quat_x         # orientation quaternion x
float64 quat_y         # orientation quaternion y
float64 quat_z         # orientation quaternion z
float64 quat_w         # orientation quaternion w
