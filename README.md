# 階段三：視覺伺服自動降落

## 1.啟動 PX4 仿真與 Gazebo 模擬環境

```
roslaunch px4 posix_sitl.launch vehicle:=iris sdf:=/home/leeching/catkin_ws/src/PX4-Autopilot/Tools/sitl_gazebo/models/iris_fpv_cam/iris_fpv_cam.sdf
```
從insert拉Apriltag36_11_00000（20cm*20cm）到地圖上
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
## 4.執行自主降落控制器
```
rosrun apriltag_ros landing_control.py
```
無人機會先繞圈飛（只是想讓它能在沒有偵測到tag的狀態，到自己飛一飛找到tag再追蹤），在相機偵測到apriltag後，調整無人機讓apriltag能在中央30cm*30cm範圍內，再直接切換成 PX4 的 AUTO.LAND 垂直降落。
## 5.開鏡頭畫面
```
rqt_image_view
```
<img width="2880" height="1800" alt="Screenshot from 2026-05-28 16-04-48" src="https://github.com/user-attachments/assets/ec9d659d-38f6-4722-b295-122b05508353" />

## 參考資料

https://github.com/AprilRobotics/apriltag_ros.git

https://github.com/koide3/gazebo_apriltag.git
