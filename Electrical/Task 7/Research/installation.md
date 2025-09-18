# INSTALLING ROS

adds ROS software soruce to your system , lsb_release gets the exact version
بيعرف السيستم بعدين ينزل الباكدجز منين
`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'` 

installs curl 
`sudo apt install curl`

بيجيب الكي (المفتاح) ل روس من اللينك و يضيفوا في السيستم انه موثوق 
similliar to API keys 
`curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`

`sudo apt update`

Full install
`sudo apt install ros-noetic-desktop-full`

بتكتب في 
bashrc 
عشان متضطرش تكتبها بنفسك كل مرة
`echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc` 
`source ~/.bashrc`

تشيك بعدها ب 
`gedit ~/.bashrc` لازم تلاقي الى انت كاتبه بعد ايكو في أخر سطر

Dependencies 
`sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`
`sudo apt install python3-rosdep`

initializing dependencies


# TURTLEBOT DEPENDENCIES
sudo apt (downloading needed packages for turtlebot as admin)


`sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers`


# TURTLEBOT3 PACKAGES

`sudo apt install ros-noetic-dynamixel-sdk`
`sudo apt install ros-noetic-turtlebot3-msgs`
`sudo apt install ros-noetic-turtlebot3`

!! FIRST CHECK BY `roscd turtlebot3` and `roscd turtlebot3_msgs` in Terminal 
لو فتح فولدر يبقي نزلوا , غير كدة عيد تاني قبل الخطوة الجاية


# GAZEBO INSTALLING
`sudo apt update`
`sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control`

test by writing `gazebo` in terminal it should open 

# SIMULATION

 Create a workspace {turtle bot} with folder src
 الباكدجز بتتحط هناك
 must be in home directory
 `mkdir -p catkin_ws/src/`



 `cd ~/catkin_ws/src/`
 `git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git`

 `git clone -b noetic https://github.com/ROBOTIS-GIT turtlebot3_manipulation_simulations.git`

 `git clone -b noetic https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git`

 `sudo apt install ros-noetic-ros-control* ros-noetic-control* ros-noetic-moveit* ros-noetic-dwa-local-planner`

 moves to workspace dic and builds needed devel and build

 `cd ~/catkin_ws && catkin_make`

catkin make should work without any errors to move forward

## OPEN MANIPULATOR


`cd ~/catkin_ws/src/`

Clone the required OpenManipulator + TurtleBot3 packages

[git clone https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_msgs.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_simulations.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_perceptions.git]


# Running in gazebo
  making it always 'waffle' in bashrc
 `echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc`
  
 reloads bashrc
 `source ~/.bashrc`

  should run model  
 `roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch`

  moves the map from downloads to another location in ws

  `mkdir -p ~/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/worlds`
  `cp /home/hod/Downloads/gamemap.sdf ~/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/worlds/`

  

# lUNCH WITH CUSTOM MAP 
   
  opens launch file
  `gedit ~/catkin_ws/src/turtlebot3_manipulation_simulations/turtlebot3_manipulation_gazebo/launch/turtlebot3_manipulation_gazebo.launch`

  ![alt text](<Screenshot from 2025-08-19 22-55-14.png>)
  `<arg name="world_name" value="/home/hod/Downloads/gamemap.sdf"/>`

  run

  `roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch`




