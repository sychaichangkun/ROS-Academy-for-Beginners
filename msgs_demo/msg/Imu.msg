#消息包含了从惯性原件中得到的数据，加速度为m/^2，角速度为rad/s
#如果所有的测量协方差已知，则需要全部填充进来
#如果只知道方差，则只填充协方差矩阵的对角数据即可

#位置：sensor_msgs/Imu.msg

Header header
Quaternion orientation
float64[9] orientation_covariance

Vector3 angular_velocity
float64[9] angular_velocity_covariance

Vector3 linear_acceleration
float64[] linear_acceleration_covariance
