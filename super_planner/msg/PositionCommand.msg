# Position command message for trajectory control
std_msgs/Header header

# Position
geometry_msgs/Point position

# Velocity
geometry_msgs/Vector3 velocity

# Acceleration
geometry_msgs/Vector3 acceleration

# Jerk
geometry_msgs/Vector3 jerk

# Yaw angle
float64 yaw

# Yaw rate
float64 yaw_dot

# Attitude (roll, pitch, yaw)
geometry_msgs/Vector3 attitude

# Angular velocity
geometry_msgs/Vector3 angular_velocity

# Thrust
geometry_msgs/Vector3 thrust

# Velocity norm
float64 vel_norm

# PID gains
float64[3] kx
float64[3] kv

# Trajectory ID
uint32 trajectory_id

# Trajectory flag
uint8 trajectory_flag
