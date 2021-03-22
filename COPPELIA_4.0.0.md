## How to Install CoppeliaSim 4.0.0 in Ubuntu 16.04 with ROS Bridge and the Velodyne plugin

Install Tutorial Coppeliasim - Espeleo Simulation

- 1 - Download CoppeliaSim V4.0.0 for Ubuntu 16.04 (https://coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz). Unzip into a suitable folder.
		
		$ wget -P /tmp https://coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
		$ cd /tmp && tar -xvf CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
		$ mv CoppeliaSim_Edu_V4_0_0_Ubuntu16_04 ~/

- 2 - Prepare ".bashrc" for CoppeliaSim:

		$ echo 'export COPPELIASIM_ROOT_DIR="$HOME/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04"' >> ~/.bashrc && source ~/.bashrc
		$ echo 'alias coppelia="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"' >> ~/.bashrc && source ~/.bashrc

- 3 - Test if the program is working on terminal by:

		$ coppelia
		
	If everything went well, you must be greeted by the CoppeliaSim simulator main window!

- 4 - Go to your catkin workspace source folder ("~/catkin_ws/src/") and clone recursively the plugin repository:

		$ cd ~/catkin_ws/src/
		$ git clone https://github.com/CoppeliaRobotics/simExtROSInterface --branch coppeliasim-v4.0.0
		
- 5 - **Fix a python requirement that is broken in CoppeliaSim 4.0.0**:

		$ cd $COPPELIASIM_ROOT_DIR/programming
		$ rm -rf libPlugin
		$ git clone https://github.com/CoppeliaRobotics/libPlugin.git
		$ cd libPlugin
		$ git checkout 1e5167079b84ca002a6197414d51c40eda583d01
		
- 6 - Install the support packages:

		$ sudo apt-get install -y python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs		

- 7 - Use "catkin build" to compile your packages. To do so, you must "catkin clean", then "catkin build"

		$ cd ~/catkin_ws
		$ catkin clean -y && catkin build

- 8 - If your compilation finished succesfully, the library "libv_repExtRosInterface.so" compiled correctly. 
	This library makes CoppeliaSim recognize the ROS enviroment in your machine. Now, copy this library to the CoppeliaSim directory:
	
		$ cp ~/catkin_ws/devel/lib/libsimExtROSInterface.so $COPPELIASIM_ROOT_DIR
		
- 9 - It's necessary to install the package Coppeliasim Plugin Velodyne which is responsible for publishing the velodyne point cloud from a C++ plugin, increasing the simulation performance.
		
		$ cd ~/catkin_ws/src/ && git clone https://github.com/ITVRoC/coppeliasim_plugin_velodyne.git
		$ cd ~/catkin_ws && catkin build
		$ cp ~/catkin_ws/devel/.private/coppeliasim_plugin_velodyne/lib/libv_repExtRosVelodyne.so $COPPELIASIM_ROOT_DIR

	The scenes in this repository already have the other configurations.
	If you want to create a new scene with the plugin follow the steps of the link - https://github.com/ITVRoC/coppeliasim_plugin_velodyne.


- 10 - Everything is ready to run. To test the communication, run the ROS master:

		$ roscore

- 11 - Now run CoppeliaSim:

		$ coppelia
		
Note that there are multiple init messages from CoppeliaSim on terminal. An indication that your library was compiled correctly is the following message:

```
		Plugin 'RosInterface': loading...
		Plugin 'RosInterface': warning: replaced variable 'simROS'
		Plugin 'RosInterface': load succeeded.
```


- 12 - To confirm the interaction between ROS and CoppeliaSim, play the empty scene in the begin of the program. In other terminal type:

		$ rostopic list
		
	If the topic "/tf" appears, the ROS/CoppeliaSim is enabled and functional.
