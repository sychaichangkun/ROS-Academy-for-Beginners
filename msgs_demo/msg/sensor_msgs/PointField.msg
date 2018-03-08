#定义点的描述类型

uint8 INT8 = 1
uint8 UINT8 = 2
uint8 INT16 = 3
uint8 UINT16 = 4
uint8 INT32 = 5
uint8 UINT32 = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name			#场的名字
uint32 offset		#点struct起始位置的偏移量
uint8 datatype	#数据类型枚举，如上
uint32 count		#场中有多少元素
