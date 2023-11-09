![YDLIDAR](image/index-X4.jpg  "YDLIDAR_X4")

YDLIDAR SDK [![Build Status](https://travis-ci.org/cansik/sdk.svg?branch=samsung)](https://travis-ci.org/cansik/sdk) [![Build status](https://ci.appveyor.com/api/projects/status/2w9xm1dbafbi7xc0?svg=true)](https://ci.appveyor.com/project/cansik/sdk) [![codebeat badge](https://codebeat.co/badges/3d8634b7-84eb-410c-b92b-24bf6875d8ef)](https://codebeat.co/projects/github-com-cansik-sdk-samsung)
=====================================================================


Introduction
-------------------------------------------------------------------------------------------------------------------------------------------------------

YDLIDAR(https://www.ydlidar.com/) series is a set of high-performance and low-cost LIDAR sensors, which is the perfect sensor of 2D SLAM, 3D reconstruction, multi-touch, and safety applications.

If you are using ROS (Robot Operating System), please use our open-source [ROS Driver]( https://github.com/ydlidar/ydlidar_ros) .

Licence
-------------------------------------------------------------------------------------------------------------------------------------------------------

The SDK itself is licensed under BSD [license](license)

Release Notes
-------------------------------------------------------------------------------------------------------------------------------------------------------
| Title      |  Version |  Data |
| :-------- | --------:|  :--: |
| SDK     |  1.4.5 |   2020-01-04  |


- [feature] Supports new and old protocol tof lidar.
- [feature] Display of version number and serial number of single communication lidar.





Dataset 
-------------------------------------------------------------------------------------------------------------------------------------------------------


| Model      |  Baudrate |  Sampling Frequency | Range(m)  | Scanning Frequency(HZ) | Working temperature(°C) | Laser power max(mW) | voltage(V) | Current(mA) | Intensity
| :-------- | --------:|--------:|  --------:| --------:|--------:| --------:| --------:| --------:| :--: |
| TG15   |  512000 |   20000  |  0.1-15   |3-15.7|0-50| ~5|4.8-5.2|400-480| false |
| TG30   |  512000 |   4000  |  0.1-30   |3-15.7|0-50| ~5|4.8-5.2|400-480| false |
| TG50   |  512000 |   4000  |  0.1-50   |3-15.7|0-50| ~5|4.8-5.2|400-480| false |
| G2A    |  230400 |   5000  |  0.12-12   |5-12|0-50| ~5|4.8-5.2|400-480| false |
| G2      |  230400 |   5000  |  0.12-12   |5-12|0-50| ~5|4.8-5.2|400-480| true |
| G2C   |  230400 |   4000  |  0.12-12   |5-12|0-50| ~5|4.8-5.2|400-480| false |

How to build YDLIDAR SDK samples
---------------

    $ git clone https://github.com/ydlidar/sdk

    $ cd sdk

    $ git checkout TOF

    $ cd ..

    $ mkdir build

    $ cd build

    $ cmake ../sdk

    $ make			###linux

    $ vs open Project.sln	###windows

How to run YDLIDAR SDK samples
---------------
    $ cd samples

linux:

    $ ./ydlidar_test
    $Please enter the lidar serial port:/dev/ttyUSB0

windows:

    $ ydlidar_test.exe
    $Please enter the lidar serial port:/dev/ttyUSB0


You should see YDLIDAR's scan result in the console:

	[YDLIDAR]:SDK Version: 1.4.5
	[YDLIDAR]:Lidar running correctly ! The health status: good
	[YDLIDAR] Connection established in [/dev/ttyUSB0][512000]:
	Firmware version: 1.3
	Hardware version: 1
	Model: TG30
	Serial: 2020010200010001
	[YDLIDAR INFO] Current Sampling Rate : 20K
	[YDLIDAR INFO] Current Scan Frequency : 15.000000Hz
	[YDLIDAR INFO] Now YDLIDAR is scanning ......
	Scan received: 1329 ranges
	Scan received: 1329 ranges
	
	



Data structure
-------------------------------------------------------------------------------------------------------------------------------------------------------
	
See [the protocol page](include/ydlidar_protocol.h) for more info.
    

Coordinate System
-------------------------------------------------------------------------------------------------------------------------------------------------------

![Coordinate](image/image.png  "Coordinate")


Upgrade Log
---------------

2020-01-04 version 1.4.5

   1.Supports new and old protocol tof lidar.

   1.Display of version number and serial number of single communication lidar.

2019-12-03 version 1.4.4

   1.Supports all standard Lidars.

2019-12-03 version 1.4.3

   1.support G4 G6 TG lidar

2019-08-29 version 1.4.2

   1.offset angle.

2019-08-12 version 1.4.2

   1.support G2 G2A G2C.

2019-05-10 version:1.4.1

   1.fix memory leak.

2019-03-25 version:1.4.0

   1.fix Large motor resistance at startup issues.

   2.fix ascendScanData timestamp issues.

   3.check lidar abnormality when turn on lidar.

   4.only support G4 lidar

   5.Remove other lidar model interfaces functions.

   6.fix turnOn function.
   
2018-12-07 version:1.3.9

   1.Remove other lidar model interfaces functions.

   2.Remove heartbeat

2018-11-24 version:1.3.8

   1.Reduce abnormal situation recovery time.
   
   2.fix timestamp from zero.

2018-10-26 version:1.3.7

   1.add input angle calibration file.
   
   2.remove network.

2018-10-15 version:1.3.6

   1.add network support.

2018-05-23 version:1.3.4

   1.add automatic reconnection if there is an exception

   2.add serial file lock.

2018-05-14 version:1.3.3

   1.add the heart function constraint.

   2.add packet type with scan frequency support.

2018-04-16 version:1.3.2

   1.add multithreading support.

2018-04-16 version:1.3.1

   1.Compensate for each laser point timestamp.
   
   
   Contact EAI
---------------

If you have any extra questions, please feel free to [contact us](http://www.ydlidar.cn/cn/contact)
