TypeDef.h是数据类型定义，移植时注意数据类型是否一致;

RxP.c/RxP.h是串口数据分析处理函数，把NMEA语句逐条存放到buffer里；
NmeaParser.c/NmeaParser.h是解析NMEA语句的函数；
CommUtil.c/CommUtil.h是一些通用功能函数；

example是示例代码，函数调用范例在example.c最下面；