# imlidar_driver
ROS imlidar Driver. This driver works to reading scans from the Inmotion's USB port.

乐行天下lidar驱动,version: Beta v1.1。
适用lidar型号：ILD26TRI
在 ubuntu1404LTS + ROS indigo 下测试通过。

编译：
在neato_edit目录下使用catkin_make命令进行编译即可。

首次使用：
1、将ILD26TRI接入USB口，运行以下指令更改USB口的权限。
请根据实际使用的端口号调整“USB_PORT”,(使用 ls /dev 命令可以查看端口号)
sudo chmod 777 /dev/USB_PORT
2、配置终端环境：
请根据实际的目录修改“PROJECT_DIRTORY”。请根据实际的shell选择.bash设置文件还是.zsh等其他类型的设置文件
source PROJECT_DIRTORY/neato_edit/devel/setup.bash
3、运行驱动：
roslaunch inmotion_lidar_driver inmotion_lidar.launch
4、在ros中发布一个静态坐标系变换，用于在rviz中观察lidar数据。这一步仅为了方便在rviz中显示lidar数据用，实际使用时并不必须。
rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map inmotion_lidar_link 100
5、运行rviz观察lidar数据效果。并添加LaserScan数据。
rosrun rviz rviz

设置调整：
打开inmotion_lidar.launch，里面可以更改各类参数
port            设备端口号，默认值“/dev/ttyUSB0”
baud_rate       串口波特率，默认值“115200”，请勿随意调整波特率
frame_id        lidar数据所相对的坐标系名称，默认值“inmotion_lidar_link”
lidar_Hz        lidar的旋转频率，默认值为7，最高可以调整为10Hz
angle_min	最小探测角度，默认值为：2*PI
angle_max	最大探测角度，默认值为：0.0
angle_increment_direction 雷达角度增量方向，默认值是“cw”，即为顺时针，此时角度增量为负。可以修改为逆时针“ccw”，角度增量为正。这个值的修改可能会导致ROS中定位算法得到的Lidar朝向和实际方向相反，可以通过倒置安装Lidar来解决反向问题。
