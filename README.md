# imlidar_driver
## Purpose
ROS imlidar Driver. This driver works to reading scans from the serial port.

## Version
ROS imlidar Driver version: Beta v0.2.0  
适用lidar型号：ILD26TRI  
在 ubuntu14.04 + ROS indigo 下测试通过

## Getting started
1. 创建并初始化工作空间  
`mkdir catkin_ws`  
`cd catkin_ws`  
`wstool init src`  
2. 获取源码  
`cd src`  
`git clone https://github.com/wenhust/imlidar_driver.git`  
3. 编译  
`catkin_make ..`  
4. 配置终端环境  
`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`  
`source ~/.bashrc`  

#### 首次使用
1. 启动ROS  
`roscore`  
2. 启动imlidar  
`rosrun imlidar_driver imlidar_node`  
或者，如需配置参数则使用：  
`roslaunch imlidar_driver imlidar.launch`  
如果提示打开端口失败，则可能是端口操作权限不够。  
`ls -l /dev/ttyUSB0`  
若显示为crw-rw----，则为其加权限使其显示为crw-rw-rw-  
`sudo chmod a+rw /dev/ttyUSB0`  
使用 ls /dev 命令可以查看端口号，此处示例端口号为ttyUSB0
3. 启动view_imlidar.launch  
`roslaunch imlidar_driver view_imlidar.launch`  
接着便可以在rviz中观察数据是否正确

#### 参数设置  
使用gedit打开imlidar.launch，里面可以更改各类参数  
port:设备端口号，默认值“/dev/ttyUSB0”
baud_rate:串口波特率，默认值“115200”，请勿随意调整波特率
frame_id:lidar数据所相对的坐标系名称，默认值“imlidar”
rps:旋转频率，默认值为7Hz，最高可以调整为10Hz
data_sequence_direction:雷达角度增量方向，默认值是“cw”，即为顺时针，此时角度增量为负。可以修改为逆时针“ccw”，角度增量为正。这个值的修改可能会导致ROS中定位算法得到的Lidar朝向和实际方向相反，可以通过倒置安装Lidar来解决反向问题。|
