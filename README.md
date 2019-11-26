# ROS_Basics
A  bunch of scripts (Python) and maybe more to cover some basics in ROS.
The idea is to explain basic functionality (nodes and launch files) all under a single package. These are based on various tutorials from across the ROS Wiki, that tries to consolidate them under relevant headings.
<br> MTC 161019


## Sections
1. [Packages](#packages)
2. [Nodes](#nodes)
3. [Launch Files](#launch-files)
4. [Further Reading](#further-reading)


These are all under the assumption that you've already created your catkin workspace. If not, follow this link [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), before proceeding further.


## Packages
Packages are directories that house nodes of similar functionality. Let's create a package `basic_ros` in the catkin workspace, as [follows](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
```
$ catkin_create_pkg basic_ros std_msgs rospy
```
then run `catkin_make` and source the bash file 
```
$ cd ~/catkin_ws && catkin_make
$ source devel/setup.bash
```
**Tip:** 
*Instead of sourcing the bash file in every new terminal, you could add it to the ~/.bashrc file. This can also be done via command line as follows*
```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```


In the above line, `std_msgs rospy` are the package dependencies. Use the `rospack` command to find all the dependencies of a ROS package. <br>


**Tip:** 
*In case you forgot to specify certain dependencies or want to add more dependencies to a package, you can do so by modifying the `package.xml` to include them using the <build_depend> and <run_depend> tags.*


## Nodes
Nodes in ROS can be compared to apps in an OS, they need to be executed to perform a certain or any operation. The most basic types of nodes are a publisher node and subscriber node, more details can be found in the [ROS Wiki](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29). 
<br>All the nodes, written in python, can be found in the **[scripts](https://github.com/mtc-20/ROS_Basics/tree/master/scripts)  folder**. Just download the entire folder into the `basic_ros` package/directory, and make sure all the files are executable
```
chmod +x talker.py
```
before executing them as follows
```
$ rosrun basic_ros talker.py
```
Of course, remember that you need to have 'roscore' running in another terminal for 'rosrun' to work. Alternatively, go to the `scripts` directory and run 
```
$ python basic_ros talker.py
```
In case this didn't work, you may have to run `catkin_make`. 


**Tip:**
*A neat shortcut to check if your package has been built properly is to use the autocomplete feature with roscd or rosrun command. Basically type the first two or three letters of your package name and double press the tab button,if the autocomplete doesn't work, it implies that you either haven't sourced the workspace or havent built your packages yet.*
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
$ rosrun basic_ros talker_param.py _freq:=3
```
> **NOTE:** *Make sure `rospy.get_param()` is declared after `rospy.init_node()`.*


## Launch Files
Each node is normally executed using the rosrun command, which implies that a new terminal has to be opened for every node required. This is circumvented by a launch file, XML files that run the different nodes serially.
<br> All the launch files are maintained in the **[launch](https://github.com/mtc-20/ROS_Basics/tree/master/launch) folder** in `basic_ros` and these are called using the `roslaunch` command.
```
$ roslaunch basic_ros chat.launch
```
- ### Chat
File - [chat.launch](https://github.com/mtc-20/ROS_Basics/blob/master/launch/chat.launch) <br>
A simple launch file that 'launches' three of the above nodes. <br>Notice nothing is output/printed to the terminal. However, using the `rostopic echo` shows that the talker are in fact publishing. The `rosnode info` tool will show all the subscriptions/publications to the corresponding nodes.


- ### Chat with Arguments
File - [chat_param.launch](https://github.com/mtc-20/ROS_Basics/blob/master/launch/chat_param.launch) <br>
A simple launch file that 'launches' the *listener* and *talker_param* nodes, accepts arguments and outputs the listener node. Notice running this launch file sets the talker_param node's default frequency to 3 instead of 0.5Hz. Arguments can be provided in command line as follows

```
$ roslaunch basic_ros chat_param.launch freq:=1
```


## PLANNED
- [ ] ROS Messages
- [x] ~~Useful tools: like rqt, roswtf, rospack, etc.~~ : shifted to a new repo, can be found [here](https://github.com/mtc-20/CheatSheets/blob/master/ROS_CS.md)

---
## FURTHER READING
- [rospy tutorials](http://wiki.ros.org/rospy_tutorials) 
