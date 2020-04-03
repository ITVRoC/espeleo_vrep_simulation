# EspeleoRobo V-REP/ROS Simulator
----------------------
This repository contains all needed files for simulating EspeleoRobo in V-REP.

## EspeleoRobo models included:
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



## Contact

Any questions, please contact-me in ``mateusnazarioc@gmail.com``.
All pull requests are welcome and desired!!!
