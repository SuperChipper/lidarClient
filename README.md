# Minimal service client cookbook recipes

This package contains a few strategies to create service clients.
The `client` recipe shows how to request data from a service with a blocking call.
The `client_async` recipe shows how to request data from a service with a non blocking call. The user has to check if a response has been received in the main loop
The `client_async_member_function` recipe is analog to `client_async` but sends the request inside a MinimalClient class
### 雷达中心指向-180,逆时针为正，共计404个点360度
```bash
source ./install/setup.bash
ros2 run lidar_client client
```
## (For windows) Bind usbport to wsl

Run cmd with administrator mode.

``` shell

usbipd list
```

![alt text](image.png)

``` shell
usbipd bind --busid 2-1
usbipd attach --wsl --busid 2-1
```

## Run ydlidar_ros2_driver

##### Run ydlidar_ros2_driver using launch file

The command format is : 

 `ros2 launch ydlidar_ros2_driver [launch file].py`

1. Connect LiDAR uint(s).
   ```
   ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
   ```
   or 

   ```
   launch $(ros2 pkg prefix ydlidar_ros2_driver)/share/ydlidar_ros2_driver/launch/ydlidar.py 
   ```
2. RVIZ 
   ```
   ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py 
   ```
    ![View](images/view.png  "View")

3. echo scan topic
   ```
   ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_client #or ros2 topic echo /scan
   ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_clientpy
   ```