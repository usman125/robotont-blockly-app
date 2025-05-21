Robotont Standalone ROS Backend
======

This code setup a ROS2 Jazzy Jalisco operating system using Ubuntu 24.04. It also contains the simulations setup for Robotont. If you have not already installed the Jazzy Jalisco on your machine. Please follow the [Jazzy Jalsico Documentation](https://docs.ros.org/en/jazzy/Installation/Alternatives/macOS-Development-Setup.html#system-requirements)

## How to setup?

```git
git clone
cd ~/robotont-blockly-app  
colcon build --packages-select my_py_pkg --symlink-install && source install/setup.bash  
ros2 launch robotont_driver fake_driver_launch.py
ros2 run turtlesim turtle_teleop_key --ros-args -r turtle1/cmd_vel:=cmd_vel
ros2 run my_py_pkg circle_controller
```
The above defined commands will run a circle controller with turtle_teleop_key node renaming the topic name with cmd_vel, and then launch RViZ simulator to view results.

In order to setup the RViZ simulator, follow the below instruction, download the following packages in your workspace.

```git
cd ~/robotont-blockly-app  
git submodule add -b jazzy-devel https://github.com/robotont/robotont_driver
git submodule add -b jazzy-devel https://github.com/robotont/robotont_msgs
git submodule add -b jazzy-devel https://github.com/robotont/robotont_description
```

Once the packages are installed, use following commands.

```
colcon build # build the code
source install/setup.bash # source the setup file
ros2 launch robotont_driver fake_driver_launch.py # launch a test robotont simulator
```

To add the new package set the dependencies of the package in `package.xml` 

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>example_interfaces</depend>
```

and also put in the `entery_points` of your `setup.py` file, and then follow how to set up guide from start. 

```python
entry_points={
  'console_scripts': [
      "circle_controller = my_py_pkg.circle_controller:main"
  ],
},
```
