Tractobots Project Setup Guide

Overview

Tractobots is a project involving field robots for autonomous navigation and operation using ROS (Robot Operating System) and MapViz for offline map visualization. This guide will walk you through setting up the required dependencies, installing ROS2, setting up Docker, and configuring ROS packages and MapViz for offline Google Maps visualization.

Prerequisites

	•	Ubuntu 20.04 LTS or 18.04 LTS (recommended)
	•	Basic knowledge of Linux command line
	•	Administrator (sudo) privileges

1. Install ROS2

ROS2 (Robot Operating System 2) is the next generation of ROS, providing enhanced features, improved security, and performance. We will install ROS2 Foxy Fitzroy, the LTS version for Ubuntu 20.04.

Step 1: Set Up the Sources

	1.	Add the ROS2 apt repository:

sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'


	2.	Update the apt package index:

sudo apt update



Step 2: Install ROS2 Foxy Desktop

	3.	Install ROS2 Foxy:

sudo apt install -y ros-foxy-desktop


	4.	Source the ROS2 setup file:

echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc



Step 3: Install ROS2 Dependencies

	5.	Install additional ROS2 tools and dependencies:

sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-argcomplete


	6.	Initialize rosdep and update:

sudo rosdep init
rosdep update



2. Install Docker

Docker is required to run the mapproxy container for offline Google Maps with MapViz. Follow the steps below to install Docker on Ubuntu.

Step 1: Set Up Docker Repository

	1.	Add Docker’s official GPG key:

sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 58118E89F3A912897C070ADBF76221572C52609D


	2.	Set up the Docker repository:

sudo add-apt-repository 'deb https://apt.dockerproject.org/repo ubuntu-xenial main'


	3.	Update the apt package index:

sudo apt update



Step 2: Install Docker Engine

	4.	Install Docker:

sudo apt install -y docker-engine


	5.	Start and enable Docker:

sudo systemctl start docker
sudo systemctl enable docker


	6.	Add your user to the Docker group to run Docker without sudo:

sudo usermod -aG docker $USER


	7.	Log out and log back in for the group changes to take effect.

3. Install ROS Packages for MapViz and Localization

MapViz is a visualization tool that integrates with ROS for displaying map data. We will install the necessary ROS packages for MapViz.

sudo apt install -y ros-foxy-mapviz ros-foxy-mapviz-plugins ros-foxy-tile-map ros-foxy-robot-localization

4. Set Up Offline Google Maps with MapViz

To visualize maps offline using MapViz, we will use the mapproxy Docker container.

Step 1: Create a MapProxy Directory

	1.	Create a directory to store MapProxy data:

mkdir ~/mapproxy



Step 2: Run MapProxy Docker Container

	2.	Run the mapproxy Docker container:

sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy



This command will start a Docker container that serves MapProxy on port 8080.

5. Install catkin_tools (Optional)

catkin_tools is a set of command-line tools for working with ROS Catkin workspaces. It’s an alternative to the standard catkin_make.

	1.	Install catkin_tools:

sudo apt install python3-catkin-tools


	2.	Clone the Tractobots repository and build it:

git clone https://github.com/kylerlaird/tractobots.git
cd tractobots
catkin build
source devel/setup.bash



6. Usage Instructions

To run the Tractobots launchers and MapViz, open different terminal windows and run the following commands:

	1.	Launch the Tractobots bringup:

roslaunch tractobots_launchers bringup.launch


	2.	Launch MapViz:

roslaunch tractobots_launchers mapviz.launch



These commands will launch the Tractobots setup and visualize the robot’s environment using MapViz.

7. Additional Notes

	•	ROS Environment: Ensure that your ROS environment is sourced properly by adding source /opt/ros/foxy/setup.bash to your .bashrc file.
	•	Docker Permissions: Ensure you have the correct permissions to run Docker commands without sudo by adding your user to the Docker group.
	•	Offline Maps: The mapproxy Docker container must be running for offline map visualization.
