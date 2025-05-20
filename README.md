Robotont Standalone ROS Backend
===

This code setup a ROS2 Jazzy Jalisco operating system using Ubuntu 24.04. It also contains the simulations setup for Robotont. If you have not already installed the Jazzy Jalisco on your machine. Please follow the [Jazzy Jalsico Documentation](https://docs.ros.org/en/jazzy/Installation/Alternatives/macOS-Development-Setup.html#system-requirements)

## How to setup?

```
git clone
cd ~/robotont-blockly-app  
colcon build --packages-select my_py_pkg --symlink-install && source install/setup.bash  
ros2 launch robotont_driver fake_driver_launch.py
ros2 run my_py_pkg circle_controller.py
```
