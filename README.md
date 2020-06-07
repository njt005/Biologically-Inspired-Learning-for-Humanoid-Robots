# Biologically Inspired Learning for Humanoid Robots

This is a collection of implementations from the biologically-inspired learning for humanoid robotics course at TUM.  Various models derived from biology were implemented to test the advantages and disadvantages when using to control a NAO robot.  All implementations were conducted in python 2.7 using ROS Kinetic installed on Ubuntu 16.04 and using the Webots simulator.  See below for detailed instructions for setup.

Setup:
1. Ubuntu 16.04 was used for all implementations: <br/>
http://releases.ubuntu.com/16.04/ <br/>

2. Several terminals are needed to be used at once.  I recommend "Terminator" as the terminal emulator since you can split the viewport and make new tabs easily. <br/>

$ sudo apt install terminator <br/>

Terminator will automatically replace the standard terminal and you can call it with the same shortcut after installation (Ctrl+Alt+T). You can find an article describing shortcuts here: <br/>
https://www.tecmint.com/terminator-a-linux-terminal-emulator-to-manage-multiple-terminal-windows/ <br/>

3. “Robot Operating System” ROS Kinetic was used for all implementations. Download and install the ROS Kinetic Desktop Full version: <br/>

https://wiki.ros.org/kinetic/Installation/Ubuntu <br/>

4. For compiling ROS code, I used “catkin tools” over the standard “catkin” that
comes bundled in ROS. Catkin tools provides a clearer interface and some quality of life functions. <br/>

$ sudo apt install python-catkin-tools <br/>

Documentation can be found here: https://catkin-tools.readthedocs.io/en/latest/ <br/>

5. For simulation, I used “Webots” for all implementations. Install the .deb file: <br/>
https://www.cyberbotics.com/ <br/>

6. NAO ROS packages for connecting to the real and virtual robot: <br/>

$ sudo apt install “ros-kinetic-nao*” <br/>

7. Download the attached pynaoqi compressed file and extract somewhere in your home directory (e.g. in ~/software/) <br/>

8. Git clone naoqisim: https://github.com/cyberbotics/naoqisim somwehere in your home directory (e.g. in ~/software/) <br/>

9. export the following paths in the .bashrc file in the home directory (~): <br/>

source /opt/ros/kinetic/setup.bash <br/>
export WEBOTS_HOME=/usr/local/webots <br/>
export "PYTHONPATH=$PYTHONPATH:$HOME/<path_to_install>/pynaoqi" <br/>
export "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/<path_to_install>/pynaoqi:$WEBOTS_HOME/lib" <br/>

Adapt the <path_to_install> to the actual path of installation <br/>

10. cd into the naoqisim directory and inside the “Makefile” comment out everything after this line: <br/>

@+make --silent -C controllers/naoqisim $(MAKECMDGOALS) <br/>

The NAO soccer functionality is not needed - just the basic controller and simulator-sdk <br/>

11. Call $ make in the naoqisim root folder and build the NAO controller <br/>

12. Webots should be able to start and with the naoqisim.wbt world. You should see a NAO robot with two camera feeds. <br/>

13. With the naoqisim world open, try to run the ROS connection: <br/>

$roslaunch nao_bringup nao_full_py.launch <br/>

This spawns several ROS nodes that interface with the NaoQi API controller that was built for Webots. Please ignore the TF error message that the transform for base_link is NaN, this is a bug in naoqisim that can be ignored for now. <br/>

14. Create a workspace by creating a folder in the ~/ros/ directory called <br/>
~/ros/bioinspired_ws/src <br/>
and use the command catkin init inside. Create a package inside the src folder by using: <br/>

$ catkin create pkg <package_name> --catkin-deps rospy std_msgs <br/>

This creates the package with whatever you put for <package_name> with dependencies on “rospy” which is the ROS python interface and “std_msgs” which contains the standard message types. The package will also contain a CmakeLists.txt and package.xml file. <br/>

Inside the package create a folder called scripts and inside put the script files from this repository (eg. put central_node.py and keyboard_node.py in the scripts folder for a package named ROS_Basics)
Make the files executable with: <br/>

$ chmod +x <filename> <br/>
  
To compile, cd into the root of the workspace <br/>

$ cd ~/ros/bioinspired_ws <br/>

and use the command <br/>

$ catkin build <br/>

To run, tell the terminal that there is new executable code by sourcing the workspace: <br/>

$ source ~/ros/bioinspired_ws/devel/setup.bash <br/>

Run the nodes by calling: <br/>

$ rosrun <package_name> <python_script.py>

