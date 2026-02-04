# speak 範例

## 建置方法

```
cd ~/
git clone https://github.com/pf-robotics/kachaka-api.git

mkidr -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s ~/kachaka-api/ros2/kachaka_interfaces .
ln -s ~/kachaka-api/ros2/demos/kachaka_speak .

cd ~/ros2_ws
colcon build
```

## 執行方法

* 在啟動 ros2_bridge 的狀態下，執行以下命令

```
cd ~/ros2_ws
source install/setup.bash
ros2 run kachaka_speak speak
```
