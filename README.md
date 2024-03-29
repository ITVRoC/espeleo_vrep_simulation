-----------
# README espeleo_vrep_simulation

# EspeleoRobo V-REP/ROS Simulation
----------------------
This repository contains all needed files for simulating EspeleoRobo in V-REP. **Tested in Ubuntu 16.04**. For other Ubuntu versions such as 18.04 this repo branch **is experimental**.

## EspeleoRobo models included:
- 6 legs
- 6 wheels
- 6 wheels plus imu, kinect and velodyne
- 6 wheels plus imu, kinect and hokuyo spinning
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
- DARPA SubT Urban Scene
- DARPA SubT Edgar Mine Scene
- walls (include espeleo with hokuyo)

## How to Install CoppeliaSim with ROS Bridge under Ubuntu 16.04

- Version 4.1.0 (recommended): [Tutorial for CoppeliaSim 4.1.0](COPPELIA_4.1.0.md)
- Version 4.0.0 (previous stable): [Tutorial for CoppeliaSim 4.0.0](COPPELIA_4.0.0.md)

## Final Considerations:

The **libv_repExtRosInterface.so**, by default, only accepts common ROS message types in CoppeliaSim. In case you wish to use other message types or even custom messages, do as indicated in the interface repository:
	 	
		
> Edit meta/messages.txt and meta/services.txt if you need to include more ROS > 
> messages/services. You need to specify the full message/service type, i.e. 
> geometry_msgs/Twist rather than Twist.

	
These files are inside the interface package. Besides this, it is necessary to add the message package dependency in  th CMakelists.txt and package.xml.

After the edition of these files to add new elements to the library, a new recompilation and copy of the library must be done. So every time you add a new message, this process must be repeated.
	
## Troubleshooting

- If you got the following error:

```
print('error: program "{0}" is missing (hint: try "sudo apt install {0}")'.format(what), file=sys.stderr)
```

Change the folder `programming/libPlugin` from your COPPELIASIM_ROOT_DIR for the following [libPlugin](https://github.com/CoppeliaRobotics/libPlugin) and compile again:


```
$ cd $COPPELIASIM_ROOT_DIR/programming
$ sudo rm -r libPlugin
$ git clone https://github.com/CoppeliaRobotics/libPlugin.git
```

- If you got any of the following errors:

![cmake_error](https://user-images.githubusercontent.com/51409770/86668418-fe5b9280-bfc8-11ea-85cd-cbf4b64655b5.png)
![cmake_error2](https://user-images.githubusercontent.com/51409770/86668488-0d424500-bfc9-11ea-8e4b-18295ae9742e.png)

Delete the previously downloaded repository.
```
sudo rm -r ~/catkin_ws/src/simExtROSInterface
```
And replace it with the old version.
```
git clone https://github.com/ITVRoC/simExtROSInterface.git
```
# Espeleo Simulation

To run the EspeleoRobo Coppelia simulation, first, clone this repository and other packages needed in your workspace:
```sh
$ cd ~/catkin_ws/
$ git clone https://github.com/ITVRoC/espeleo_vrep_simulation.git
$ git clone https://github.com/ITVRoC/espeleo_locomotion.git
$ git clone https://github.com/ITVRoC/espeleo_description.git
$ git clone https://github.com/ITVRoC/espeleo_bringup.git
$ git clone https://github.com/ITVRoC/espeleo_msg_srv.git
```

Also clone or install via apt the ROS Web Video Server:
```sh
sudo apt install ros-$ROS_DISTRO-web-video-server (RECOMMENDED)
or
git clone https://github.com/RobotWebTools/web_video_server.git
```

### Espeleo Locomotion
The espeleo_locomotion package is responsible for control the movement of the robot, sending the RPM for each wheel, according to the kinematic model used. This package is also responsible for loading some mechanical parameters used in the simulation, like the value of each wheel reduction.   The simulation will not run without those parameters.

### Espeleo Description
Espeleo description package is responsible for the EspeleoRobo TF tree. The simulation only provides the frames of each sensor and the relation of the "base_link" frame to the "world" frame. 

### Espeleo Bringup
Espeleo bringup is responsible to start the dynamic reconfigure server, allowing the use of some functionalities like "turbo button" and changing the direction of the movement. 

### Espeleo Msg and Srv
Espeleo messages and services contains espeleo's messages and services data structures.

### Web Video Server
Required package to convert ROS Streams to HTTP, allowing to use the robot's cameras in simulation in Espeleo's GUI.


## Optional packages

### Espeleo GUI

Espeleo_gui is a control interface that allows the user to get the motor's current feedback, front and back camera streams, record videos, bags and change some parameters of EspeleoRobo. 
```sh
$ git clone https://github.com/ITVRoC/espeleo_gui.git
```

### Espeleo Teleop
 
This package has nodes especially adapted that allow the use of a keyboard or joystick to control EspeleoRobo. 
```sh
$ git clone https://github.com/ITVRoC/espeleo_teleop.git
```


## How to run
Use these commands to run the simulation:

```sh
$ roscore
$ coppelia
```

In Coppelia, open the  EspeleoRobo scene or model. Select "**File**" and "**Open new scene**".

Search for "**_espeleo_6wheel_ITV.ttt_**", located in Folder : "**espeleo_vrep_simulation/vrep_models/scenarios/ITVRoC/**

Then:

```sh
$ roslaunch espeleo_vrep_simulation espeleo_sim.launch
```

 And press play in the simulation. 
 
 You can launch the GUI with the following command:
 ```sh
$ roslaunch espeleo_gui gui_simulation.launch
```


## Contact

Any questions, please contact ``hector.azpurua@itv.org``.
Pull requests are welcome (and desired)!
