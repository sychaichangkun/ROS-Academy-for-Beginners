#消息描述了2D网络地图，每一个cell的值都代表这点被占用的概率
#位置：nav_msgs/OccupancyGrid.msg

Header header
MapMetaData info
int8[] data #地图网格被占用的概率，数值在[0-100]之间，未知区域设置为-1
