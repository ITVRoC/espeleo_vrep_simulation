-----------
# README espeleo2_vrep

# EspeleoRobo V-REP/ROS Simulator
----------------------
This repository contains all needed files for simulating EspeleoRobo in V-REP.

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

## How to Install CoppeliaSim with ROS Bridge

Install Tutorial Coppeliasim - Espeleo Simulation

- 1 - Download CoppeliaSim (CoppeliaSim Edu 18.04 or 16.04, according to your linux version) - https://coppeliarobotics.com/downloads . Unzip in a suitable folder.

- 2 - Add a line on your .bashrc, indicating your CoppeliaSim root path (Path from previous step):


		$ export COPPELIASIM_ROOT_DIR="<path_to_coppeliasim_root_dir>"
	
	
	Save, close e reload the .bashrc:
	
	
		$ source ~/.bashrc

- 3 - Create an alias of the simulator to your terminal:


```sh
	$ echo "alias coppelia=$COPPELIASIM_ROOT_DIR/coppeliaSim.sh" >> ~/.bashrc
	$ source ~/.bashrc
```

- 4 - Test if the program is working on terminal by:


		$ coppelia

- 5 - Go to your "../catkin_ws/src/" and clone recursively the plugin repository:


		$ git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface
		

- 6 - Install the support packages:


		$ sudo apt-get install python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs
		

- 7 - Use "catkin build" to compile your packages. To do so, you must "catkin clean", then "catkin build"

```sh
	$ catkin clean		
	$ catkin build
```

- 8 - If your compilation finished succesfully, the library "libv_repExtRosInterface.so" compiled correctly. 
	This library makes CoppeliaSim recognize the ROS enviroment in your machine. Now, copy this library to the CoppeliaSim directory:
	
	
		$ cp ~/catkin_ws/devel/lib/libsimExtROSInterface.so $COPPELIASIM_ROOT_DIR
		
		
- 9 - Everything is ready to run. To test the communication, run the ROS master:


		$ roscore

- 10 - Now run CoppeliaSim:


		$ coppelia
		
		
Note that there are multiple init messages from CoppeliaSim on terminal. An indication that your library was compiled correctly is the following message:


```
		Plugin 'RosInterface': loading...
		Plugin 'RosInterface': warning: replaced variable 'simROS'
		Plugin 'RosInterface': load succeeded.
```


- 11 - To confirm the interaction between ROS and CoppeliaSim, play the empty scene in the begin of the program. In other terminal type:


		$ rostopic list
		
		
	If the topic "/tf" appears, the ROS/CoppeliaSim is enabled and functional.
	
	

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
$ git clone https://github.com/ITVRoC/espeleo_vrep_simulation.git
$ git clone https://github.com/ITVRoC/espeleo_locomotion.git
$ git clone https://github.com/ITVRoC/espeleo_description.git
$ git clone https://github.com/ITVRoC/espeleo_bringup.git
$ git clone https://github.com/ITVRoC/espeleo_msg_srv.git
$ git clone https://github.com/ITVRoC/coppeliasim_plugin_velodyne.git
```
Also clone or install via apt the ROS Web Video Server:
```sh
sudo apt install ros-<distro>-web-video-server (RECOMMENDED)
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

### Coppeliasim Plugin Velodyne
Coppeliasim Plugin Velodyne is responsible for publishing the velodyne point cloud from a C++ plugin, increasing the simulation performance.

#### To use
After cloning this repository and compiling with `catkin make` or `catkin build`, the plugin lib needs to be copied into the CoppeliaSim folder:

```sh
$ cp ~/catkin_ws/devel/.private/coppeliasim_plugin_velodyne/lib/libv_repExtRosVelodyne.so $COPPELIASIM_ROOT_DIR
```

The scenes in this repository already have the other configurations.

If you want to create a new scene with the plugin follow the steps of the link - https://github.com/ITVRoC/coppeliasim_plugin_velodyne.


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
After compiling all the needed packages, go to espeleo_vrep_simulation and change the branch to espeleo2_vrep and compile the package:
```sh
$ roscd espeleo_vrep_simulation
$ git checkout espeleo2_vrep
$ catkin build espeleo_vrep_simulation
```

Then, use these commands to run the simulation:

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

Any questions, please contact-me in ``mateusnazarioc@gmail.com``.
All pull requests are welcome and desired!!!
