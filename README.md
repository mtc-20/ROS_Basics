# ROS_Basics
A  bunch of scripts and maybe more to cover some basics in ROS.
The idea is to create basic functionality (nodes and launch files) all under a single package.
<br> MTC 0919
<br>

## Packages
Packages are directories that house nodes of similar functionality. Let's create a package `basic_ros` in the catkin workspace, as [follows](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
```
catkin_create_pkg basic_ros std_msgs rospy roscpp
```
then run `catkin_make` and source the bash file.

## Nodes
Nodes in ROS can be compared to apps in an OS, they need to be executed to perform a certain or any operation. The most basic types of nodes are a publisher node and subscriber node, more details can be found in the [ROS Wiki](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29). 
<br>All the nodes, written in python, can be found in the **[scripts](https://github.com/mtc-20/ROS_Basics/tree/master/scripts)  folder**. Just download the entire folder into the `basic_ros` package/directory, and make all the files are executable
```
chmod +x talker.py
```
before executing them as follows
```
rosrun basic_ros talker.py
```

- ### Talker
File - [talker.py](https://github.com/mtc-20/ROS_Basics/blob/master/scripts/talker.py) <br>
A simple publisher node that publishes a string message, "hello world" with a time stamp, at the rate of 1 per second to the *talker* topic.

- ### Listener
File - [listener.py](https://github.com/mtc-20/ROS_Basics/blob/master/scripts/listener.py) <br>
A simple subscriber node that subscribes to the *talker* and *bingo* topics and prints the message to the log file. Notice running the `listener.py` node without any active publisher nodes doesn't produce any output.

- ### Bingo
File - [call_bingo.py](https://github.com/mtc-20/ROS_Basics/blob/master/scripts/call_bingo.py) <br>
Another publisher node that publishes a random number between 1 and 100 as a string message to the *bingo* topic.

- ### Talker with Parameters
File - [talker_param.py](https://github.com/mtc-20/ROS_Basics/blob/master/scripts/talker_param.py) <br>
A modified talker node that now accepts a ROS parameter to modify it's frequency of printing mesages to the server. By default, it will print 1 message every 2 seconds(0.5Hz). This parameter can be changed to 3Hz via command line as follows
```
rosrun basic_ros talker_param.py _freq:=3
```
**NOTE:** *Make sure `rospy.get_param()` is declared after `rospy.init_node()`.*
## Launch Files
Each node is normally executed using the rosrun command, which implies that a new terminal has to be opened for every node required. This is circumvented by a launch file, XML files that run the different nodes serially.
<br> All the launch files are maintained in the **[launch](https://github.com/mtc-20/ROS_Basics/tree/master/launch) folder** in `basic_ros` and these are called using the `roslaunch` command.
```
roslaunch basic_ros chat.launch
```
- ### Chat
File - [chat.launch](https://github.com/mtc-20/ROS_Basics/blob/master/launch/chat.launch) <br>
A simple launch file that 'launches' three of the above nodes. <br>Notice nothing is output/printed to the terminal. However, using the `rostopic echo` shows that the talker are in fact publishing. The `rosnode info` tool will show all the subscriptions/publications to the corresponding nodes.


- ### Chat with Arguments
File - [chat_param.launch](https://github.com/mtc-20/ROS_Basics/blob/master/launch/chat_param.launch) <br>
A simple launch file that 'launches' the *listener* and *talker_param* nodes, accepts arguments and outputs the listener node. Notice running this launch file sets the talker_param node's default frequency to 3 instead of 0.5Hz. Arguments can be provided in command line as follows

```
roslaunch basic_ros chat_param.launch freq:=1
```
