#消息包含了一张未经压缩的图像，图像原点定义在(0,0)，即图片左上方
Header header       #时间戳位接收图像消息的时间，坐标id是相机光学坐标系原点
uint32 height       #图像高度，整数，表示图像行数
uint32 width        #图像宽度，整数，表示图像列数
string encoding     #像素编码方式
uint8 is_bigendian  #数据是否按照字节顺序
uint32 step         #行长
uint8[] data        #真实矩阵数据，矩阵尺寸为(step*rows)
