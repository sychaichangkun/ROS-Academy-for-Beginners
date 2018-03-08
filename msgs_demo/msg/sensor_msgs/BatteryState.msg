#描述电池状态，健康状态等
#电池充电状态相关参数
uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
uint8 POWER_SUPPLY_STATUS_CHARGING = 1
uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
uint8 POWER_SUPPLY_STATUS_FULL = 4

#电池充电健康状态相关参数
uint8 POWER_SUPPLY_HEALTH_UNKNOWN = 0
uint8 POWER_SUPPLY_HEALTH_GOOD = 1
uint8 POWER_SUPPLY_HEALTH_OVERHEAT = 2
uint8 POWER_SUPPLY_HEALTH_DEAD = 3
uint8 POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4
uint8 POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5
uint8 POWER_SUPPLY_HEALTH_COLD = 6
uint8 POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7
uint8 POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8

#电池技术(化学属性)相关属性参数
uint8 POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0	#电池属性未知
uint8 POWER_SUPPLY_TECHNOLOGY_NIMH = 1		#镍氢电池
uint8 POWER_SUPPLY_TECHNOLOGY_LION = 2		#锂离子电池
uint8 POWER_SUPPLY_TECHNOLOGY_LIPO = 3		#锂聚合物电池
uint8 POWER_SUPPLY_TECHNOLOGY_LIFE = 4		#锂铁电池
uint8 POWER_SUPPLY_TECHNOLOGY_NICD = 5		#镍镉电池
uint8 POWER_SUPPLY_TECHNOLOGY_LIMN = 6		#锂锰电池

Header header
float32 voltage				#电压V
float32 current				#当不充电时为负值，如果不在测量则为NaN
float32 charge				#所充电量，单位Ah，如果不测量则为NaN
float32 capacity			#容纳电量，单位Ah，如未测量则为NaN
float32 design_capacity		#设计容纳电量,单位Ah,如未测量则为NaN
float32 percentage			#充电百分比,范围[0-1],如未测量则为NaN
uint8 power_supply_status	#电池充电状态，其值定义如上
uint8 power_supply_health	#电池健康状态，其值定义如上
uint8 power_supply_technology	#电池化学属性，其值定义如上
bool present				#如果检测到有电池则为True

float32[] cell_voltage		#每个单独cell的电压所组成的数组
string location				#电池安装位置
string serial_number		#电池序列号
