YDLIDAR ROS驱动包(V1.4.5)
=====================================================================



怎么构建ROS驱动包
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    	(1). git clone https://github.com/YDLIDAR/ydlidar_ros
    	(2). git chectout gaussian
    2) Running catkin_make to build ydlidar_node and ydlidar_client
    3) Create the name "/dev/ydlidar" for YDLIDAR
    --$ roscd ydlidar_ros/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh

怎样运行YDLIDAR ROS 包
=====================================================================
有两种方式运行方式

1. 运行YDLIDAR 节点并在RVIZ上显示
------------------------------------------------------------
	roslaunch ydlidar_ros lidar_view.launch

2.运行YDLIDAR 节点在终端上显示
------------------------------------------------------------
	roslaunch ydlidar_ros lidar.launch

	rosrun ydlidar_ros ydlidar_client



配置参数
=====================================================================
port (string, default: /dev/ydlidar)

    当前雷达端口号

baudrate (int, default:230400)

    当前雷达波特率（G2 G2A: 230400, G2C:115200)

frame_id (string, default: laser_frame)

    雷达坐标系名称

reversion (bool, default: true)

    是否旋转雷达数据180度

resolution_fixed (bool, default: true)

    是否固定角度分辨率输出
auto_reconnect(bool, default: true)

     是否雷达支持热插拔

angle_min (double, default: -180)

    雷达最小有效角度

angle_max (double, default: 180)

    雷达最大有效角度

range_min (double, default: 0.08)

    雷达最小有效距离

range_max (double, default: 16.0)

    雷达最大有效距离

ignore_array (string, default: "")

    剔除角度列表

frequency (double, default: 10)

    雷达扫描频率

samp_rate (int, default: 5)

    雷达采样频率

isSingleChannel(bool, default: false)

    是否是单通讯雷达(S2, X2, X2L, TX8， TX20)

isTOFLidar(bool, default: false)

    是否是单通信TOF雷达(TG15 TG30,TG50, TX8, TX20)




更新日志
=====================================================================

2020-01-04 version:1.4.5

  1.支持新旧协议TOF雷达(版本号小于1.3是旧协议雷达）

  1.支持单通道雷达获取序列号和版本号信息

2019-12-03 version:1.4.4

  1.支持所有标品序列雷达

2019-12-03 version:1.4.3

  1.支持G4, G6,G1, TG序列雷达

2019-07-16 version:1.4.2

  1.扫描频率偏移

2019-03-25 version:1.4.1

   1.修复内存泄露.

2019-03-25 version:1.4.0

   1.修复时间戳错误
   
   2.雷达启动异常检测
   
   3.移除别的雷达型号支持, 仅仅支持G2A雷达
   
   4.优化turnOn 和 turnOff 
