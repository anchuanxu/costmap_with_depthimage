### costmap层插件使用须知


1. drop_layers

该插件需要先编译xbot_msgs包之后再进行编译，目前将其CMakeLists.txt中相关行注释。


### 所需要安装依赖

运行`install_deps.sh`脚本


### 启动xbot_navi包最新更新地址

https://yt.droid.ac.cn/slam/slam_navi/-/tree/xbot_plus

### 启动点云转激光实现相机避障

```
roslaunch pointcloud_to_laserscan sample_node.launch 
```

注意确认相机发布的点云topic是否和sample_node.launch 中的一致，
如果不一致，请修改后启动。

### 更新

由于点云转激光在tf过程中，消耗大量计算资源导致降低帧数，无法达到实时性要求，现在使用depthImagetolaserscan包进行处理相机信息。