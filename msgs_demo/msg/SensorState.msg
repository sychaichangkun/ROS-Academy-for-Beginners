# Xbot Sensor Data Messages
#except IMU sensor,echo sensors and infrared

# Over current states, the sum of each channel represents they are over current at the same time, for example, 15 indicates that front_left,front_right,rear_left,rear_right four channels are all over current as 15=1+2+4+8
uint8 OVER_CURRENT_FRONT_LEFT_WHEEL  = 1
uint8 OVER_CURRENT_FRONT_RIGHT_WHEEL = 2
uint8 OVER_CURRENT_REAR_LEFT_WHEEL = 4
uint8 OVER_CURRENT_REAR_RIGHT_WHEEL = 8
uint8 OVER_CURRENT_UP_DOWN_WHEEL = 16


###### MESSAGE ######

Header header

###################
# Timestamp and battery voltage
###################
uint16 time_stamp      # milliseconds starting when turning on Xbot (max. 65536, then starts from 0 again)
float32  battery_voltage         # battery voltage in float (24V~29.5V)

###################
# 5 channel encoders
###################
uint16 front_left_encoder    # accumulated ticks left wheel starting with turning on Xbot (max. 65535)
uint16 front_right_encoder   # accumulated ticks right wheel starting with turning on Xbot (max. 65535)
uint16 rear_left_encoder
uint16 rear_right_encoder
uint16 up_down_encoder


###################
# 5 channel current
###################
float32 front_left_current
float32 front_right_current
float32 rear_left_current
float32 rear_right_current
float32 up_down_current

###################
# over current indicator
###################
uint8   over_current   # see over current states

