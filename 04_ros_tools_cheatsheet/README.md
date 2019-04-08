# ROS tools cheatsheet

`ROS` comes with a lot of useful utilities which make your life developing and operating `ROS` enabled robots so much easier. Here is a list of the utilities I have compiled whilst hacking on `ROS`

# catkin

`catkin` is `ROS` build cli utility

## create workspace
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## create new package
```shell
catkin_create_pkg <pkg_name> <dep1> <dep2>...
catkin_create_pkg agitr
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

## build a specific package
```shell
catkin_make --pkg <package_name>
```
## build all packages
```shell
catkin_make
```

# rostopic

`rostopic` provides information about `ROS` topics

## list node topics
```shell
rostopic list
/rosout
/rosout_agg
```

## dump/copy messages published to a topic to stdout
```shell
rostopic echo <topic_name>
```

## topic bandwidth i.e. message rate in bytes/s
```shell
rostopic bw <topic_name>
```

## topic message rate i.e. messages/s
```shell
rostopic hz <topic_name>
```

## query topic info
```shell
rostopic info /rosout
Type: rosgraph_msgs/Log.  (MESSAGE TYPE here is Log)

Publishers: None

Subscribers:
 * /rosout (http://06752f9c886c:34891/)
```

## publish message on a topic with a given rate
```shell
rostopic pub -r <rate> <topic_name> <message_type> <message_content>
```

## publish one message and exit
```shell
rostopic pub -1 <topic_name> <message_type> <message_content>
```

## publish message on a topic from a [bag] file (YAML)
```shell
rostopic pub -f <filename> <topic_name>
```

## find topics that publish certain message
```shell
rostopic find std_msgs/Int32
```

# rosmsg

`rosmsg` provides information about `ROS` messages

## list all messages
```shell
rosmsg list
```

## list messages defined in some package
```shell
rosmsg package package_name
```

## list packages that contain some messages
```shell
rosmsg packages
```

## query message info (by addressing the message Type)
```shell
rosmsg show rosgraph_msgs/Log
```

## display message md5sum
```shell
rosmsg md5 <message>
```

# rosservice

`rosservice` provides information about `ROS` services

## list running services
```shell
rosservice list
```

## list nodes that offer service
```shell
rosservice node /rosout/get_loggers
```

## list data type of service
```shell
rosservice info /rosout/get_loggers
```

## call a service
```shell
rosservice call /rosout/get_loggers
```

## query service type
```shell
rosservice type /rosout/get_loggers
```

## query service arguments (req message)
```shell
rosservice args /rosout/get_loggers
```

## set log level (loglevel, loggerlevel)
```shell
rosservice call /node_name/set_logger_level ros.package_name LOGLEVEL
**this is a service call: the node must be running in order for it to succeed!**
```

# rossrv

`rossrv` displays information about `ROS` service types

## query service data type
```shell
rossrv info roscpp/GetLoggers
```

## inspect service data type
```shell
rossrv show Spawn
rossrv show turtlesim/Spawn
```

# rosdep

`rosdep` handles `ROS` dependencies

## initialise ROS dependencies (must be RUN ONLY ONCE -- after the installation)
```shell
sudo rosdep init
```

## update all ROS system packages (DON'T RUN AS ROOT!)
```shell
rosdep update
```

## install and build package
```shell
rosdep install <package_name>
```

## install all packages dependencies for all packages in your workspace (CD TO THE ROOT OF WORKSPACE)
```shell
rosdep install --from-paths src --ignore-src -r -y
```

## resolve dependency after adding to rosdep rules
```shell
rosdep update
rosdep resolve my_dependency_name
```

# rosbag

`rosbag` allows to record and replay `ROS` messages

## record messages published on a topic to a file
```shell
rosbag record -O <filename> <topic>
```

## record all messages on every topic currently being published
```shell
rosbag record -a
```

## record messages but enable compression (BZ2)
```shell
rosbag record -j -O <filename> <topic>
rosbag record --lz4 -O <filename> <topic>  ->>> (lz4 compression)
```

## record messages for duration and then split after the limit
```shell
rosbag record --duration=DURATION --split -O <filename> <topic>
```

## replay stored messages
```shell
rosbag play <filename>
```

## query message information
```shell
rosbag info <filename>
```

# rosclean

`rosclean` handles various cleanup tasks such as checking the size and cleanup of the logs

## check log size
```shell
rosclean check
272K ROS node logs
```

## remove (purge) logs
```shell
rosclean purge
```

# rosnode

`rosnode` allows to manage and provides information about `ROS` nodes

## list running nodes
```shell
rosnode list
/rosout
```

## get information about node
```shell
rosnode info <node_name>
```

## kill nodes
```shell
rosnode kill <node_name>
```

## cleanup dead nodes
```shell
rosnode cleanup
```

# rospack

`rospack` allows to manage `ROS` packages

## list installed packages
```shell
rospack list
```

## find package
```shell
rospack find rosout
/opt/ros/melodic/share/rosout
```

## list package dependencies
Immediate:
```shell
rospack depends1 <package name>
```
All deps:
```shell
rospack depends <package name>
```

# rosls, roscd

`rosls` lists files in `ROS` filesystem hierarchy;
`roscd` allows to change your working directory within `ROS` filesystem hierarchy

## list package directory
```shell
rosls rosout
cmake  package.xml
```

```shell
rosls rosout/cmake
rosoutConfig-version.cmake  rosoutConfig.cmake
```

## cd to package dir
```shell
roscd rosout
root@ros:/opt/ros/melodic/share/rosout# pwd
/opt/ros/melodic/share/rosout
```

# roslaunch

`roslaunch` allows to launch and manage multiple `ROS` nodes at once

## launch a group of nodes
```shell
roslaunch <package-name> <launch-filename> [args]
Or full path:
roscd roslaunch && roslaunch example.launch
roslaunch pr2_robot/pr2_bringup/pr2.launch
```

## force all nodes to log into console
```shell
roslaunch --screen <package-name> <launch-filename> [args]
```

## list all nodes in launch file
(enable included launch file; this overrides run_foo2 arg)
```shell
roslaunch --nodes -v subpose main.launch run_foo2:=1
/foo2/publish_message
/foo2/subscribe_to_twist
/foo1/publish_message
/foo1/subscribe_to_twist
```

```shell
(disable included launch file)
roslaunch --nodes -v subpose main.launch run_foo2:=0
/foo1/publish_message
/foo1/subscribe_to_twist
```

## list all launch files
```shell
roslaunch --files subpose/main.launch
/root/catkin_ws/src/subpose/ns2.launch
/root/catkin_ws/src/subpose/main.launch
```

## list ROS args
```shell
roslaunch --ros-args subpose/main.launch
Optional Arguments:
  run_foo2 (default "0"): undocumented
```

```shell
roslaunch --ros-args subpose main.launch
```

## list dependencies
```shell
roslaunch-deps -w subpose/main.launch
Dependencies:
pubvel subpose

Missing declarations:
subpose/manifest.xml:
  <depend package="pubvel" />
```

## check files
```shell
rosrun roslaunch roslaunch-check main.launch
```

# rosparam

`rosparam` manages `ROS` parameters

## list parameters
```shell
rosparam list
/rosdistro
/rosversion
/run_id
```

## read specific parameter
```shell
rosparam get /run_id
```

## read all namespace parameters
```shell
rosparam get /
```

## set parameter
```shell
rosparam set /foo bbar
```

## delete parameter
```shell
rosparam delete /publish_message/max_vel
...
```

```shell
rosparam get /publish_message/max_vel
ERROR: Parameter [/publish_message/max_vel] is not set
```

## save all parameters in namespace in a file (probably YAML)
```shell
rosparam dump filename namespace
```

## load parameters from a file (probably YAML)
```shell
rosparam load filename namespace
```

# tail logs
```shell
rosparam get /run_id
```

```shell
ls -1 ~/.ros/log/<run_id>
master.log
roslaunch-06752f9c886c-8256.log
rosout-1-stdout.log
rosout.log
```
