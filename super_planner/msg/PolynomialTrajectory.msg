# Polynomial trajectory message for MPC control
std_msgs/Header header

# Trajectory type flags
uint8 POSITION_TRAJ = 1
uint8 YAW_TRAJ = 2
uint8 HEART_BEAT = 4

uint8 type

# Time information
float64 start_wt_pos  # Start world time for position trajectory
float64 start_wt_yaw  # Start world time for yaw trajectory
float64 final_wt_pos  # Final world time for position trajectory
float64 final_wt_yaw  # Final world time for yaw trajectory

# Piece/segment count
uint32 piece_num_pos
uint32 piece_num_yaw

# Polynomial order
uint32 order_pos  # Order of position polynomial
uint32 order_yaw  # Order of yaw polynomial

# Duration
float64 duration_pos
float64 duration_yaw

# Time arrays for each piece
float64[] time_pos
float64[] time_yaw

# Coefficients for x, y, z position (each dimension has piece_num_pos * (order_pos+1) coefficients)
float64[] coef_pos_x
float64[] coef_pos_y
float64[] coef_pos_z

# Coefficients for yaw (has piece_num_yaw * (order_yaw+1) coefficients)
float64[] coef_yaw

# Trajectory ID
uint32 trajectory_id
