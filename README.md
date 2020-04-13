# EspeleoRobô CoppeliaSim/ROS Simulator
----------------------
This repository contains the models to simulate EspeleoRobô in CoppeliaSim (the old V-REP).

## Firs step:

Make sure you have CoppeliaSim properly installed. Please go to the README of the package https://github.com/ITVRoC/sim_ros_interface. Follow the instructions therein come back here.

**Test if everything is working:**

Clone these packages in your catkin workspace `src` (example: `~/catkin_ws/src`) folder:

- espeleo_control: `git clone https://github.com/ITVRoC/espeleo_control.git`
- espeleo_decawave: `git clone https://github.com/ITVRoC/espeleo_decawave.git`
- espeleo_localization: `git clone https://github.com/ITVRoC/espeleo_localization.git`
- espeleo_planning: `git clone https://github.com/ITVRoC/espeleo_planning.git`
- imu_complemenntary_fiter: `git clone https://github.com/ITVRoC/imu_complementary_filter.git`
- ros_eposmcd (note: this should already installed installed): `git clone https://github.com/ITVRoC/ros_eposmcd.git`

Compile:

`catkin build`

Resource your workspace:

`source ~/catkin_ws/devel/setup.bash`

After running rosmaster (`roscore`), go to your `src` folder and run CoppeliaSim with a defined scene:

`coppeliasim ~/catkin_ws/src/espeleo_vrep_simulation/vrep_models/scenarios/terrain_tree/terrain_tree.ttt`

Play the simulator ad run a simple keyboard controller to check if everything is working:

`roslaunch espeleo_vrep_simulation keyboard.launch`

Note: Replace `~/catkin_ws` if you have a different path to your catkin workspace.

## EspeleoRobô models included:
- 6 legs
- 6 wheels
- 6 star shaped wheels
- 4 wheels and 2 legs
- 4 wheels 2 legs plus imu and kinect
- 4 wheels 2 legs plus imu and velodyne

## Scenes included:
- tube (cylinder) plus espeleorobo
- terrain plus tree plus espeleorobo
- paleotoca
- campinho (Motocross field)
- DARPA SubT Challenge scene

## List of nodes

- `imu_basic_node.py` This node captures information from the gyro and the accelerometer and publishes a imu raw topic. OBS: the complementary filter in the package `imu_complementary_filter` should also be used to generate the quaternion data of the IMU.

- `pose_constructor_sim.cpp` This node publishes a pose topic of the espeleorobo. It is based on the and on the data from the simulated imu and from the `\tf`. This is an outdated code. Use `state_estimator_espeleo.cpp` from the package `espeleo_localization` instead. OBS: the complementary filter in the package `imu_complementary_filter` should also be used to generate the quaternion data of the IMU.

- `xsens_emulator.py` This node simulates the imu and the GPS data of the real espeleorobo. It is a better version of the node above node. If your code works only with `imu_basic_node.py` (togueger with `\tf` data) you should update it to work with the `xsens_emulator.py` node, since it better replicates the real robot. This node receives three parameters for setting the initial global position of the xsens, they are: (i) LAT in degrees, (ii) LON in degrees, (iii) ALT in meters above sealevel.
OBS: the complementary filter in the package `imu_complementary_filter` should also be used to generate the quaternion data of the IMU.

- `config_rviz_basic.py` Simple script to configure rviz

- `config_rviz_cylinder.py` Simple script to configure rviz in the solid cylinder scenario


## List of launch files

Note: Before running these launch files it is necessary to run ROS, then vrep and load the desired scene.

- `basic.launch` Launch the codes to control the espeleorobo with the vector field approach. After running this the controller will wait for a path to be followed

- `keyboard.launch` Launch the codes to control the espeleorobo with the keyboard


## How to interact

The simulated models behave like the real robot, emulating its topics and services in ROS.

**Topics:**
- `/ros_eposmcd/position_movement_absolute`  (message type:`ros_eposmcd_msgs/movement`): Sends relative position commands to a chosen motor in **[rad]**.
- `/ros_eposmcd/position_movement_relative`  (message type:`ros_eposmcd_msgs/movement`): Sends absolute position commands to a chosen motor in **[rad]**.
- `/ros_eposmcd/velocity_movement`  (message type:`ros_eposmcd_msgs/movement`): Sends velocity commands to a chosen motor in **[rad/s]**.

- `/sensors/acc`  (message type:`geometry_msgs/Point`): Data from the accelerometer simulated inside vrep.
- `/sensors/gyro`  (message type:`geometry_msgs/Point`): Data from the gyro simulated inside vrep.
- `/sensors/velodyne`  (message type:`sensor_msgs/PointCloud`): Data from the ouster laser simulated inside vrep.
- `/imu/raw`  (message type:`sensor_msgs/Imu`): Message that simulates a imu raw data (accelerometer and gyro)
- `/imu/data`  (message type:`sensor_msgs/Imu`): Message that simulates a imu data, with orientation quaternion
- `/cmd_vel`  (message type:`geometry_msgs/Twist`): Velocity commands for the robot, linear forward velocity and angular velocity around z axis.
- `/espeleo/pose`  (message type:`geometry_msgs/Pose`): Message that contains a pose estimation for the espeleorobo


**Services**
- `/vrep_ros_interface/ros_eposmcd/request_position`  (message type:`ros_eposmcd_msgs/maxon_telemetry`): Request a desired motor angular position in **[rad]**.
- `/vrep_ros_interface/ros_eposmcd/request_velocity`  (message type:`ros_eposmcd_msgs/maxon_telemetry`): Request a desired motor velocity in **[rad/s]**.


## Contact

Any questions, please contact-me in ``adrianomcr18@gmail.com``.




<!--

## V-REP

- The models are natively compatible with V-REP 3.6.1 (rev. 1). Retrocompatibility may work, but is not officially supported.
- You can find how to install V-REP and configure the ROS interface in  [our wiki](https://github.com/ITVRoC/general-wiki/wiki).

## Configuring this package

Before configuring this package, it is necessary that you have already configured your **ROS**, **V-REP** and **vrep_ros_interface** environments.  A quick tutorial is available in the previous section.

1- Place this repository's folder inside ``../catkin_ws/src/``.

2- Clone also inside ``../catkin_ws/src/`` the following ROS package: https://github.com/ITVRoC/ros_eposmcd .

3- If you do not have CAN driver files installed, delete the folder ``../src/ros_eposmcd/ros_eposmcd_driver``.

4- Add the following lines:
- in ``../src/vrep_ros_interface/meta/messages.txt`` you should add ``ros_eposmcd_msgs/movement``.
- in ``../src/vrep_ros_interface/meta/services.txt`` you should add ``ros_eposmcd_msgs/maxon_telemetry``.

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


## V-Rep x CoppeliaSim problems

- If you switch from V-Rep to the new CoppeliaSim version, make sure that your EspeleoRobô's lua script contains the following line `if (moduleName=='ROSInterface') then`, instead of `if (moduleName=='ROSInterface') then`. The difference is just the capital letters in ROS.

-->
