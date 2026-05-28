# 階段三：視覺伺服自動降落

## 1.啟動 PX4 仿真與 Gazebo 模擬環境

```
roslaunch px4 posix_sitl.launch vehicle:=iris sdf:=/home/leeching/catkin_ws/src/PX4-Autopilot/Tools/sitl_gazebo/models/iris_fpv_cam/iris_fpv_cam.sdf
```
- 從insert拉Apriltag36_11_00000到地圖上
```
pxh> commander arm
pxh> commander takeoff
```
## 2.建立 MAVROS 與飛控連線橋樑
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
## 3.啟動 AprilTag 視覺辨識節點
```
roslaunch apriltag_ros continuous_detection.launch
```
## 4.執行自主精準降落控制器
```
rosrun apriltag_ros landing_control.py
```
## 5.開鏡頭畫面
```
rqt_image_view
```
