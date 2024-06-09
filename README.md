Setting Up TurtleBot3 Simulation in ROS 2 Humble Hawksbill
**Requirements -
a. Ubuntu 22.04 (Jammy Jellyfish)
b. ROS 2 Humble Hawksbill
**
**Set up the ROS 2 Environment Variables**
source /opt/ros/humble/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

source ~/.bashrc

**Install ROS 2 Dependent Packages
 Cartographer:**
 sudo apt install ros-humble-cartographer 
sudo apt install ros-humble-cartographer-ros

**Navigation Stack for ROS 2**

sudo apt install ros-humble-navigation2 
sudo apt install ros-humble-nav2-bringup

**Clone the Turtlebot3 packages:**
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble-devel

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble-devel

colcon build 

source ~/turtlebot3_ws/install/setup.bash

source ~/turtlebot3_ws/install/setup.bash

echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc

**Launching Turtlebot3 Simulation World in Gazebo**
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py

**Running Turtlebot3 Teleoperation Node**
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
