# EspeleoRobo V-REP/ROS Simulator
----------------------
This repository contains all needed files for simulating EspeleoRobo in V-REP.

## EspeleoRobo models included:
- 6 legs
- 6 wheels
- 6 star shaped wheels
- 4 wheels and 2 legs

## How to interact

The simulated models behave like the real robot, emulating its topics and services in ROS.

**Topics:**
- `/ros_eposmcd/position_movement_absolute`  (message type:`ros_eposmcd_msgs/movement`): Sends relative position commands to a chosen motor in **[rad]**.
- `/ros_eposmcd/position_movement_relative`  (message type:`ros_eposmcd_msgs/movement`): Sends absolute position commands to a chosen motor in **[rad]**.
- `/ros_eposmcd/velocity_movement`  (message type:`ros_eposmcd_msgs/movement`): Sends velocity commands to a chosen motor in **[rad/s]**.

**Services**
- `/vrep_ros_interface/ros_eposmcd/request_position`  (message type:`ros_eposmcd_msgs/maxon_telemetry`): Request a desired motor angular position in **[rad]**.
- `/vrep_ros_interface/ros_eposmcd/request_velocity`  (message type:`ros_eposmcd_msgs/maxon_telemetry`): Request a desired motor velocity in **[rad/s]**.

## V-REP

- The models are natively compatible with V-REP 3.6.1 (rev. 1). Retrocompatibility may work, but is not officially supported.
- You can find how to install V-REP and configure the ROS interface in  [our wiki](https://github.com/ITVRoC/general-wiki/wiki).

## Configuring this package

Before configuring this package, it is necessary that you have already configured your **ROS**, **V-REP** and **vrep_ros_interface** environments.  A quick tutorial is available in the previous section.

1- Place this repository's folder inside ``../catkin_ws/src/``.

2- Clone also inside ``../catkin_ws/src/`` the following ROS package: https://github.com/ITVRoC/ros_eposmcd .

3- If you do not have CAN driver files installed, delete the folder ``../src/ros_eposmcd/ros_eposmcd_driver``.

4- Add the following lines:
-- in ``../src/vrep_ros_interface/meta/messages.txt`` you should add ``ros_eposmcd_msgs/movement``.
-- in ``../src/vrep_ros_interface/meta/services.txt`` you should add ``ros_eposmcd_msgs/maxon_telemetry``.

5- In `../src/vrep_ros_interface/CMakeLists.txt`, add the ``ros_eposmcd_msgs`` dependecy inside:
e.g.:
```
set(PKG_DEPS
    roscpp
    rosconsole
    (...)
    ros_eposmcd_msgs)
```

6- Additionally, insert in ``../src/vrep_ros_interface/package.xml`` the line:
```
<depend>ros_eposmcd_msgs</depend>
```

7- Compile again your **catkin_ws** tree:
```
$ catkin build
```

8- Copy the file `../catkin_ws/devel/lib/libv_repExtRosInterface.so` to you V-REP root.

9- Now, run the ROS master, open V-REP, load any EspeleoRobô model, play the scene and check if the topics have appeared, indicating that you are good to go!

## Contact

Any questions, please contact-me in ``f.rocha41@gmail.com``.
All pull requests are welcome and desired!!!
