sudo apt install -y ros-noetic-desktop-full 

source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" > ~/.bashrc


sudo apt install -y ros-noetic-rtabmap-ros
sudo apt install -y ros-noetic-catkin*
sudo pip install -U catkin_tools
sudo -H pip3 install modern_robotics
sudo apt install -y python3-rosdep
sudo apt install -y python3-rosinstall
sudo apt install -y python3-rosinstall-generator
sudo apt install -y python3-wstool
sudo apt install -y build-essential
sudo apt install -y ros-noetic-moveit
sudo apt install -y python3-matplotlib
sudo apt install -y ros-noetic-soem
sudo apt install -y ros-noetic-socketcan-interface
sudo apt install -y ros-noetic-costmap-queue
sudo apt install -y ros-noetic-mir-robot 
sudo apt install -y python3-pymodbus 
sudo apt install -y ros-noetic-realsense2-camera

sudo rosdep init
rosdep update

ubuntu_version="$(lsb_release -r -s)"
ROS_NAME="noetic"
MAESTRO_FOLDER=~/maestro_ws

if [ ! -d "$MAESTRO_FOLDER/src" ]; then
	mkdir -p $MAESTRO_FOLDER/src
	cd $MAESTRO_FOLDER/src
	
fi

cd $MAESTRO_FOLDER/src

##UR5
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git

git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
rm -rf universal_robot/ur3*
rm -rf universal_robot/ur1*
##MIR
git clone https://github.com/dfki-ric/mir_robot

##GRIPPER
git clone https://github.com/ros-industrial/robotiq
rm -rf robotiq/robotiq_3f*

##Maestro itself
git clone https://github.com/AD-SDL/sdl_maestro

cd ..

rosdep install --from-paths src --ignore-src -r -y

catkin_make

echo "source $MAESTRO_FOLDER/devel/setup.bash" >> ~/.bashrc
source $MAESTRO_FOLDER/devel/setup.bash
