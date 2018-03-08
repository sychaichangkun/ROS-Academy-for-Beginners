#本消息保存了一个N维点，可能还会包含一些附加信息，如模、强度信息等等

Header header			#时间戳是传感器得到数据的时间，坐标系是传感器坐标系

#定义点云的2D结构，如果云是无顺序的，则height为1，宽度就是点云的长度
uint32 height
uint32 width

PointField[] fields

bool is_bigendian		#数据是否是字节型的
uint32 point_step		#一个点的字节长度
uint32 row_step			#一行总共的字节长度
uint8[] data			#实际的点数据，大小是(row_step*height)

bool is_dense			#如果没有无效点，则设置为True
