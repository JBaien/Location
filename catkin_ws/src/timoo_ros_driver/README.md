# timoo  ROS驱动

#### 1. 代码获取方式
```bash 
# 克隆主仓库
git clone git@192.168.9.120:timoo-software/timoo_ros_driver.git
# 进入主仓库目录
cd timoo_ros_driver
# 初始化并拉取子模块代码
git submodule init  
git submodule update  
```
#### 2.ros驱动编译和运行

```bash 
# 1, 修改雷达类型
# 打开/launch目录下timoo.launch文件, 配置lidar_type参数（<param name="lidar_type" value="TIMOO16" type="str" />）
# 例如： TIMOO16, TIMOO32, TIMOO1550, TIMOO1550STD, TIMOO128, TIMOO128V
# 2, 编译
catkin_make
# 3, 激活环境
source devel/setup.bash
# 4, 启动驱动
roslaunch timoo_ros_driver timoo.launch
# 5, 启动rviz
rviz -f timoo
```
