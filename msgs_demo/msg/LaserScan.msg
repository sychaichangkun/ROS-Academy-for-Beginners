#平面内的激光测距扫描数据，注意此消息类型仅仅适配激光测距设备
#如果有其他类型的测距设备(如声呐)，需要另外创建不同类型的消息

#位置：sensor_msgs/LaserScan.msg

Header header			#时间戳为接收到第一束激光的时间
float32 angle_min		#扫描开始时的角度(单位为rad)
float32 angle_max		#扫描结束时的角度(单位为rad)
float32 angle_increment	#两次测量之间的角度增量(单位为rad)

float32 time_increment	#两次测量之间的时间增量(单位为s)

float32 scan_time		#两次扫描之间的时间间隔(单位为s)

float32 range_min		#距离最小值(m)
float32 range_max		#距离最大值(m)

float32[] ranges		#测距数据(m,如果数据不在最小数据和最大数据之间，则抛弃)

float32[] intensities	#强度，具体单位由测量设备确定，如果仪器没有强度测量，则数组为空即可
