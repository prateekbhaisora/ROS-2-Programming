# ROS-2-Programming

This repo contains Python3 and CPP code written for ROS-2 Humble Hawksbill LTS on Ubuntu 22.04.4 LTS running over VMware Fusion on arm64 architecture.  

So, to clear the original build and create a new fresh build, run the following commands:

```sh
mkdir ros2_ws
cd ros2_ws/
git clone https://github.com/prateekbhaisora/ROS-2-Programming.git
cd ROS-2-Programming/
mv * ../
cd ..
sudo rm -rf ROS-2-Programming/
cd build/
sudo rm -rf my_cpp_pkg/ my_py_pkg/ my_robot_bringup/ my_robot_interfaces/
colcon build --symlink-install --cmake-clean-first
```

Make sure you have all the dependencies such as pip3, python3, colcon, etc., required for ROS2, pre-installed before using this code.