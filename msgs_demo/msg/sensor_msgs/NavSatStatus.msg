#描述卫星定位状态，包括定位系统和定位成功与否

int8 STATUS_NO_FIX = -1			#无法定位
int8 STATUS_FIX = 0				#非增强定位
int8 STATUS_SBAS_FIX = 1		#星基增强定位
int8 STATUS_GBAS_FIX = 2		#地基增强定位

int8 status						#定位状态，其值为上面定义

#定位所使用的卫星导航系统
uint16 SERVICE_GPS = 1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4		#包括北斗
uint16 SERVICE_GALILEO = 8

uint16 service
