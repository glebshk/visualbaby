# IMAV-videostream
Verbindung zwischen RPI und Groundstation mit Video + Debug Ansicht

## ROS Node starten
# 1.
```
colcon build --symlink-install
```
# 2.
 ```
 source install/setup.bash
```
# 3. 

```
ros2 run video-stream stream
```

## ROS Node create
# Python
```
ros2 pkg create --build-type ament_python my_python_pkg
```