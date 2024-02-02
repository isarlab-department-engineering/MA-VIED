Header header

# Rear Left Wheel Speed 
float64 rear_left_wheel_speed
# Rear Right Wheel Speed 
float64 rear_right_wheel_speed
# Front Left Wheel Speed 
float64 front_left_wheel_speed
# Front Right Wheel Speed 
float64 front_right_wheel_speed

# Front right Wheel odometry (m)
int32   encoder_pulses_RR
uint64  encoder_pulses_RR_ts  # DAQ timestamp
# Front left Wheel odometry (m)
int32   encoder_pulses_RL
uint64  encoder_pulses_RL_ts # DAQ timestamp  

# Steering Angle ??deg
float64 steering_angle

# delta time can frames about wheels speed
# float64 dt

# Throttle
float64 throttle
# Brake
float64 brake

# Brake pressure
float64 brake_pressure

# Brake pressed state
uint8 brake_is_pressed



uint8 NO_COMMAND        = 0
uint8 NEUTRAL           = 1
uint8 FORWARD           = 2
uint8 REVERSE           = 3

uint8 EMERGENCY_STOP    = 4


uint8 actual_gear
