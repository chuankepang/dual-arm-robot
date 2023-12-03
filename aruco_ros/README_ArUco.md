###1、启动realsense节点

roslaunch realsense2_camera rs_camera.launch

###2、不指定参数，使用默认参数执行单个ArUco码检测

roslaunch aruco_ros single.launch

###3、可视化识别效果

rosrun image_view image_view image:=/aruco_single/result

###4、查看位姿数据

rostopic echo /aruco_single/pose

###5、参数服务器
 roslaunch easy_handeye publish.launch
