sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update


sudo apt install -y build-essential
sudo apt install -y python3-pip
sudo apt install -y ros-noetic-desktop-full 

source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" > ~/.bashrc


sudo pip install -U catkin_tools

sudo -H pip3 install modern_robotics

sudo apt install -y python3-rosdep
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
rm -rf universal_robot/ur5_*

##MIR
git clone https://github.com/dfki-ric/mir_robot

##GRIPPER
sudo apt install -y python3-pymodbus ##or maybe change TCP / RTU to have the correct dependencies
git clone https://github.com/ros-industrial/robotiq
rm -rf robotiq/robotiq_3f*

##Maestro itself
git clone https://github.com/AD-SDL/sdl_maestro

cd ..

rosdep install --from-paths src --ignore-src -r -y

catkin_make

echo "source $MAESTRO_FOLDER/devel/setup.bash" >> ~/.bashrc
source $MAESTRO_FOLDER/devel/setup.bash
