This is a collection of implementations from the biologically-inspired learning for humanoid robotics course at TUM.  Various models derived from biology were implemented to test the advantages and disadvantages when using to control a NAO robot.  All implementations were conducted in python 2.7 using ROS Kinetic installed on Ubuntu 16.04 and using the Webots simulator.  See below for detailed instructions for setup.

Setup:
1. Ubuntu 16.04 was used for all implementations
http://releases.ubuntu.com/16.04/
2. Several terminals are needed to be used at once.  I recommend "Terminator" as the terminal emulator since you can split the viewport and make new tabs easily.
$ sudo apt install terminator
Terminator will automatically replace the standard terminal and you can call it with the same shortcut after installation (Ctrl+Alt+T). You can find an article describing shortcuts here:
https://www.tecmint.com/terminator-a-linux-terminal-emulator-to-manage-multiple-terminal-windows/
3. “Robot Operating System” ROS Kinetic was used for all implementations.
ros-kinetic-desktop-full:
https://wiki.ros.org/kinetic/Installation/Ubuntu
4. For compiling ROS code, I used “catkin tools” over the standard “catkin” that
comes bundled in ROS. Catkin tools provides a clearer interface and some quality of life functions.
$ sudo apt install python-catkin-tools
Documentation can be found here: https://catkin-tools.readthedocs.io/en/latest/
5. For simulation, I used “Webots” for all implementations. Install the .deb file:
https://www.cyberbotics.com/
6. NAO ROS packages for connecting to the real and virtual robot:
$ sudo apt install “ros-kinetic-nao*”
7. Download the attached pynaoqi compressed file and extract somewhere in your home directory (e.g. in ~/software/)
8. Git clone naoqisim: https://github.com/cyberbotics/naoqisim somwehere in your home directory (e.g. in ~/software/)
9. export the following paths in the .bashrc file in the home directory (~):
source /opt/ros/kinetic/setup.bash
export WEBOTS_HOME=/usr/local/webots
export "PYTHONPATH=$PYTHONPATH:$HOME/<path_to_install>/pynaoqi"
export "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/<path_to_install>/pynaoqi:$WEBOTS_HOME/lib"
Adapt the <path_to_install> to the actual path of installation
10. cd into the naoqisim directory and inside the “Makefile” comment out everything after this line:
@+make --silent -C controllers/naoqisim $(MAKECMDGOALS)
The NAO soccer functionality is not needed - just the basic controller and simulator-sdk
11. Call $ make in the naoqisim root folder and build the NAO controller
12. Webots should be able to start and with the naoqisim.wbt world, you should see a NAO robot with two camera feeds.
13. With the naoqisim world open, try to run the ROS connection:
$roslaunch nao_bringup nao_full_py.launch
This spawns several ROS nodes that interface with the NaoQi API controller that was built for Webots. Please ignore the TF error message that the transform for base_link is NaN, this is a bug in naoqisim that can be ignored for now.
14. Create a workspace by creating a folder in the ~/ros/ directory called
~/ros/bioinspired_ws/src
and use the command catkin init inside. Create a package inside the src folder by using:
$ catkin create pkg <package_name> --catkin-deps rospy std_msgs
This creates the package with whatever you put for <package_name> with dependencies on “rospy” which is the ROS python interface and “std_msgs” which contains the standard message types. The package will also contain a CmakeLists.txt and package.xml file.
Inside the package create a folder called scripts and inside put the script files from this repository (eg. Put central_node.py and keyboard_node.py in the scripts folder for a package named ROS_Basics)
Make the files executable with:
$ chmod +x <filename>
To compile, cd into the root of the workspace
$ cd ~/ros/bioinspired_ws
and use the command
$ catkin build
To run, tell the terminal that there is new executable code by sourcing the workspace:
$ source ~/ros/bioinspired_ws/devel/setup.bash
Run the nodes by calling
$ rosrun <package_name> <python_script.py>

