# ROS Setup Instructions + Python Pub Sub 
>⚡**It is HIGHLY suggested, to watch the first 4-5 videos in [this ROS YouTube tutorial playlist](https://www.youtube.com/playlist?list=PLAjUtIp46jDcQb-MgFLpGqskm9iB5xfoP) if you are not familiar with ROS basic concepts⚡**

## Installing Ubuntu Focal Fossa on a VM
1. Download Ubuntu Focal Fossa from [here](https://releases.ubuntu.com/20.04/)
2. Install Ubuntu on a Virtual Machine ([guide in italian for Virtual Box](https://www.youtube.com/watch?v=62-hJarauK4&list=PLhlcRDRHVUzR-5TKDC1VPMtyhEyyQ5uwy)).
Allocate at least 25 GB of disk space and install with the minimal installation option.
3. Update Ubuntu `sudo apt-get update && sudo apt-get upgrade`.
4. Tweak VM settings to [improve performances](http://www.rawinfopages.com/tips/2017/07/speed-up-virtualbox-in-windows/).

## Installing ROS 1 Noetic
### 1. Configure Ubuntu repo
Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse."
The simplest way to do it is go on the Software and Updates app in the menù, and search and check the options that contain the words restricted, multiverse and universe. 
You can follow the [Ubuntu guide for instructions on doing this](https://help.ubuntu.com/community/Repositories/Ubuntu).

Open a terminal and imput those commands:
    
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

### 2. Set up keys
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
### 3. Installation of the full version of ROS Noetic (recommended)
    sudo apt update
    sudo apt install ros-noetic-desktop-full
### 4. Env. Setup
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
This will ensure that every time you open a shell the command `source /opt/ros/noetic/setup.bash` is executed automatically.
### 5. Install dependencies for building packages
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    
    #initialize rosdep
    sudo rosdep init
    rosdep update

    #install catkin
    sudo apt install python3-catkin-tools python3-osrf-pycommon

Refer to [this guide](http://wiki.ros.org/noetic/Installation/Ubuntu) for additional info.
Also thsi [video tutorial for ROS installation](https://youtu.be/PowY8dV36DY).


## Useful Links
- [ROS Wiki](http://wiki.ros.org/Documentation)
- [ROS Beginner Python Tutorial (YT Playlist)](https://www.youtube.com/playlist?list=PLAjUtIp46jDcQb-MgFLpGqskm9iB5xfoP)
- [Project example + video tutorial](https://github.com/utra-robosoccer/Tutorials-2020)

## Using the ROS packages in this repository
### 1. Clone this repository
Clone this repo using

    git clone https://github.com/e-candeloro/Python-ROS-pub-sub.git

### 2. Build the workspace with catkin
Move inside the folder `ROS/python_example_ros/`. This is called the "workspace" folder.
If you are in the repo main folder:
    
    cd ROS/python_example_ros
Then use catkin:

    catkin_make

After the following command, two new folders named "build" and "devel" should appear in the "python_example_ros" directory.

### 3. Initialize roscore and usage of devel/setup.bash
Every time you open a new terminal and  want to work with launch files and packages inside the workspace, you need to execute the command: `source devel/setup.bash`.

### 4. Initialize a roscore
Open a new terminal, and type:

    roscore

Leave this terminal running roscore till the end of your simulation.
You can kill it with CTRL + C when you need to stop it.
When executing ros nodes, one and only one roscore needs to be running!

### 5. Open the package you want to use or .roslaunch file you need

In our case, with the Python publisher and subscriber, we want to do the following:

1. Start the publishing node:
 
        source devel/setup.bash
        rosrun python_pub_sub publisher_node.py

    The node also logs in the terminal the data that is publishing

3. Start the subscriber node (on a new terminal!)
        
        source devel/setup.bash
        rosrun python_pub_sub subscriber_node.py
    
    You should now see, in the terminal, the message the subscriber is receiving while the publisher is speaking.

4. Kill the python publisher node by CTRL + C in the terminal.
5. Try to launch the other publisher_node_v2.py
 
        source devel/setup.bash
        rosrun python_pub_sub publisher_node_v2.py
    This node uses the `ros_publisher.py` class to offer more flexibility when publishing messages, especially if you need to publish one or in some cases!
    You should see the subriber receiving data with a different message.

In other cases, follow the same procedure but for other .roslaunch or nodes in other packages.


## Creating **from scratch** a ROS Project/Workspace with a Package  using two Python scripts for pub-sub
Watch the first 5 videos of [this playlist](https://youtube.com/playlist?list=PLAjUtIp46jDcQb-MgFLpGqskm9iB5xfoP) for a step by step tutorial.

We need to setup the ROS workspace using catkin and then we can start to add ROS packages to the workspace/project.
We can think of a ROS workspace like the trunk of a tree and of ROS packages as the tree branches.
Every ROS project contains at least one package.
A ROS package can contain libraries, datasets, config files but most importantly a ROS node.

### 1. Creating a ROS Project
    #create a directory with a src sub-directory

    mkdir -p my_ros_workspace/src
    cd my_ros_workspace/

    #this will create two more folders "devel" and "build" to make a workspace

    catkin_make
### 2. Create a ROS Package
    cd src/
    catkin_create_pkg my_ros_package [dependecies here (optional)]

In our case, with the pub-sub example, we write:
    
    catkin_create_pkg my_ros_package rospy std_msgs

### 3. Exploring the package.xml and CMakeLists.txt files
These two files, located in `my_ros_workspace/src/my_ros_package`, describe the configurations and the dependencies of the package.

### 4. Adding Python scripts in a package
Create a `scripts` folder inside the `my_ros_package` directory:
    
    cd my_ros_package/
    mkdir scripts
Then create two python files and make them executable:
    
    cd scripts
    touch publisher_node.py
    touch subscriber_node.py
    chmod +x *
Open the CMakeLists.txt in the `my_ros_package` folder and, after the part
    
    catkin_package( 
    #  INCLUDE_DIRS include
    #  LIBRARIES python_pub_sub
    #  CATKIN_DEPENDS rospy std_msgs
    #  DEPENDS system_lib
    )
write, in a new line the following:

    catkin_install_python(PROGRAMS scripts/publisher_node.py scripts/subscriber_node.py
    DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
    )
and save.


### 5. Writing and running the pub-sub package
You can now work on the python pub-sub scripts.
You can copy the `publisher_node.py` and  `subscriber_node.py` scripts from this repo, in the `python_example/python_pub_sub/scripts` folder.

- For the publisher, follow [this video tutorial](https://youtu.be/yqYvMEYJoTk).
- For the subscriber, follow [this video tutorial](https://youtu.be/MkiWj4VwZjc).

For running the scripts, follow the procedures in in the previous sections, using `roscore`, `source devel/setup.bash`, `roslaunch ros_package_name roslaunchfile.roslauch`, `rosrun ros_package_name rosnode.py`.

