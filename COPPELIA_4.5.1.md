## How to Install CoppeliaSim 4.5.1 in Ubuntu 20.04 with ROS Bridge and the Velodyne plugin

Install Tutorial Coppeliasim - Espeleo Simulation

- 1 - Download CoppeliaSim V4.5.1 for Ubuntu 20.04 (https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04.tar.xz). Unzip into a suitable folder.
		
		$ wget -P /tmp https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04.tar.xz
		$ cd /tmp && tar -xvf CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04.tar.xz
		$ mv CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04 ~/

- 2 - Prepare ".bashrc" for CoppeliaSim:

		$ echo 'export COPPELIASIM_ROOT_DIR="$HOME/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04"' >> ~/.bashrc && source ~/.bashrc
		$ echo 'alias coppelia="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"' >> ~/.bashrc && source ~/.bashrc

- 3 - Test if the program is working on terminal by:

		$ coppelia
		
	If everything went well, you must be greeted by the CoppeliaSim simulator main window!

- 4 - Go to your catkin workspace source folder ("~/catkin_ws/src/") and clone recursively the plugin repository. **Note: the simExtROSInterface that works with Ubuntu 16.04 is the coppeliasim-v4.0.0 branch. Until now, we have seen no problems.**

		$ cd ~/catkin_ws/src/
		$ git clone https://github.com/CoppeliaRobotics/simExtROS.git
		
- 5 – Install necessary dependencies:

  $ sudo apt install xlstproc
  $ python3 -m pip install xmlschema

- 6 - Use "catkin build" to compile your packages. To do so, you must "catkin clean", then "catkin build"

		$ cd ~/catkin_ws
		$ catkin clean -y && catkin build

- 7 - If your compilation finished succesfully, the library "libv_repExtRosInterface.so" compiled correctly. 
	This library makes CoppeliaSim recognize the ROS enviroment in your machine. Now, copy this library to the CoppeliaSim directory:
	
		$ cp ~/catkin_ws/devel/lib/libsimExtROS.so $COPPELIASIM_ROOT_DIR
		
- 8 - It's necessary to install the package Coppeliasim Plugin Velodyne which is responsible for publishing the velodyne point cloud from a C++ plugin, increasing the simulation performance.
		
		$ cd ~/catkin_ws/src/ && git clone https://github.com/ITVRoC/coppeliasim_plugin_velodyne.git
		$ cd ~/catkin_ws && catkin build
		$ cp ~/catkin_ws/devel/.private/coppeliasim_plugin_velodyne/lib/libv_repExtRosVelodyne.so $COPPELIASIM_ROOT_DIR

	The scenes in this repository already have the other configurations.
	If you want to create a new scene with the plugin follow the steps of the link - https://github.com/ITVRoC/coppeliasim_plugin_velodyne.


- 9 - Everything is ready to run. To test the communication, run the ROS master:

		$ roscore

- 10 - Now run CoppeliaSim:

		$ coppelia


- 11 - To confirm the interaction between ROS and CoppeliaSim, play the empty scene in the begin of the program. In other terminal type:

		$ rostopic list
		
	If the topic "/tf" appears, the ROS/CoppeliaSim is enabled and functional.
