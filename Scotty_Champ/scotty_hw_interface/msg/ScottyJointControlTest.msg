string[] joint_names
float32[] joint_angles
float32[] kp_values
float32[] kd_values
float32[] velocity_values
float32[] torque_values
bool centralized_mode
bool[] centralized_enable_flags  # [abad, hip, knee, kp, kd, vel, trq]
float32[] centralized_values     # Values to apply when centralized_mode is enabled