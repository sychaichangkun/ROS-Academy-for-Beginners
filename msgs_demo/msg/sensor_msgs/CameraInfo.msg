#文件描述了从相机导出的原始信息，消息应当在一个camera_info的topic上
Header header	#时间坐标系统,Header时间戳为接收图像的时间,Header坐标系id相机的光学坐标系

#相机标定参数，将相机拍摄到的原始图像(扭曲图像)转化成正常图像
#包括1. 非扭曲图像(需要D和K)； 2.调整图像(需要D,K,R)

#定义图像尺寸
uint32 height
uint32 width

#定义矫正模型，支持的模型列表在文件sensor_msgs/distortion_models.h中给出；对于大多数
#相机来说，“plumb_bob”模型就已经足够
string distortion_model

#扭曲模型相关参数，大小取决于模型，对于“plumb_bob”模型来说,5各参数分别为(k1,k2,t1,t2,k3).
float64[] D

#相机变换矩阵,将相机坐标系中的3D坐标点利用焦距(fx,fy)和主点(cx,cy)转换到2D像素坐标系内
#矩阵形式为
#   [fx  0  cx]
# K=[0  fy  cy]
#   [0   0   1]
float64[9]  K  #一个3*3的矩阵

#立体照片的调整矩阵
float64[9] R	#3*3的矩阵

#相机投影矩阵，包括旋转和平移
#     [fx'  0  cx'  Tx]
# P = [ 0  fy' cy'  Ty]
#     [ 0   0   1    0]

float64[12] P	#3*4的矩阵

#定义图像装箱参数，用以将多个像素合并为一个像素，可以降低图像分辨率
#其值为0时等同于为1，表示不对图像进行合并像素
uint32 binning_x
uint32 binning_y

#定义感兴趣区域
RegionOfInterest roi
