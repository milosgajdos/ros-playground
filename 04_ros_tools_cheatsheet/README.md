# ROS tools cheatsheet

`ROS` comes with a lot of useful utilities which make your life developing and operating `ROS` enabled robots so much easier. Here is a list of the utilities I have compiled whilst hacking on `ROS`

# catkin

`catkin` is `ROS` build cli utility

## create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

## create new package
catkin_create_pkg <pkg_name> <dep1> <dep2>...
catkin_create_pkg agitr
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

## build a specific package
catkin_make --pkg <package_name>

## build all packages
catkin_make

# rostopic

`rostopic` provides information about `ROS` topics

## list node topics
rostopic list
/rosout
/rosout_agg

## dump/copy messages published to a topic to stdout
rostopic echo <topic_name>

## topic bandwidth i.e. how many bytes/s are being published + other basic stats (min, max, mean)
rostopic bw <topic_name>

## topic message rate i.e. how many messages/s are being published + other stats (min, max, stddev)
rostopic hz <topic_name>

## query topic info
rostopic info /rosout
Type: rosgraph_msgs/Log.  (MESSAGE TYPE here is Log)

Publishers: None

Subscribers:
 * /rosout (http://06752f9c886c:34891/)

## publish message on a topic with a given rate
rostopic pub -r <rate> <topic_name> <message_type> <message_content>

## publish one message and exit
rostopic pub -1 <topic_name> <message_type> <message_content>

## publish message on a topic from a [bag] file (YAML)
rostopic pub -f <filename> <topic_name>

## find topics that publish certain message
rostopic find std_msgs/Int32

# rosmsg

`rosmsg` provides information about `ROS` messages

## list all messages
rosmsg list

## list messages defined in some package
rosmsg package package_name

## list packages that contain some messages
rosmsg packages

## query message info (by addressing the message Type)
rosmsg show rosgraph_msgs/Log

## display message md5sum
rosmsg md5 <message>

# rosservice

`rosservice` provides information about `ROS` services

## list running services
rosservice list

## list nodes that offer service
rosservice node /rosout/get_loggers

## list data type of service
rosservice info /rosout/get_loggers

## call a service
rosservice call /rosout/get_loggers

## query service type
rosservice type /rosout/get_loggers

## query service arguments (req message)
rosservice args /rosout/get_loggers

## query service data type
rossrv info roscpp/GetLoggers

## inspect service data type
rossrv show Spawn
rossrv show turtlesim/Spawn

## set log level (loglevel, loggerlevel)
rosservice call /node_name/set_logger_level ros.package_name LOGLEVEL
**this is a service call: the node must be running in order for it to succeed!**

# rosdep

`rosdep` handles `ROS` dependencies

## initialise ROS dependencies (must be RUN ONLY ONCE -- after the installation)
sudo rosdep init

## update all ROS system packages (DON'T RUN AS ROOT!)
rosdep update

## install and build package
rosdep install <package_name>

## install all packages dependencies for all packages in your workspace (CD TO THE ROOT OF WORKSPACE)
rosdep install --from-paths src --ignore-src -r -y

## resolve dependency after adding to rosdep rules
rosdep update
rosdep resolve my_dependency_name

# rosbag

`rosbag` allows to record and replay `ROS` messages

## record messages published on a topic to a file
rosbag record -O <filename> <topic>

## record all messages on every topic currently being published
rosbag record -a

## record messages but enable compression (BZ2)
rosbag record -j -O <filename> <topic>
rosbag record --lz4 -O <filename> <topic>  ->>> (lz4 compression)

## record messages for duration and then split after the limit
rosbag record --duration=DURATION --split -O <filename> <topic>

## replay stored messages
rosbag play <filename>

## query message information
rosbag info <filename>

# rosclean

`rosclean` handles various cleanup tasks such as checking the size and cleanup of the logs

## check log size
rosclean check
272K ROS node logs

## remove (purge) logs
rosclean purge

# rosnode

`rosnode` allows to manage and provides information about `ROS` nodes

## list running nodes
rosnode list
/rosout

## get information about node
rosnode info <node_name>

## kill nodes
rosnode kill <node_name>

## cleanup dead nodes
rosnode cleanup

# rospack

`rospack` allows to manage `ROS` packages

## list installed packages
rospack list

## find package
rospack find rosout
/opt/ros/melodic/share/rosout

## list package dependencies
Immediate:
rospack depends1 <package name>
All deps:
rospack depends <package name>

# rosls, roscd

`rosls` lists files in `ROS` filesystem hierarchy;
`roscd` allows to change your working directory within `ROS` filesystem hierarchy

## list package directory
rosls rosout
cmake  package.xml

rosls rosout/cmake
rosoutConfig-version.cmake  rosoutConfig.cmake

## package manifest
<pkg_dir>/package.xml

## cd to package dir
roscd rosout
root@ros:/opt/ros/melodic/share/rosout# pwd
/opt/ros/melodic/share/rosout

# roslaunch

`roslaunch` allows to launch and manage multiple `ROS` nodes at once

## launch a group of nodes
roslaunch <package-name> <launch-filename> [args]
Or full path:
roscd roslaunch && roslaunch example.launch
roslaunch pr2_robot/pr2_bringup/pr2.launch

## force all nodes to log into console
roslaunch --screen <package-name> <launch-filename> [args]

## list all nodes in launch file
(enable included launch file; this overrides run_foo2 arg)
roslaunch --nodes -v subpose main.launch run_foo2:=1
/foo2/publish_message
/foo2/subscribe_to_twist
/foo1/publish_message
/foo1/subscribe_to_twist

(disable included launch file)
roslaunch --nodes -v subpose main.launch run_foo2:=0
/foo1/publish_message
/foo1/subscribe_to_twist

## list all launch files
roslaunch --files subpose/main.launch
/root/catkin_ws/src/subpose/ns2.launch
/root/catkin_ws/src/subpose/main.launch

## list ROS args
roslaunch --ros-args subpose/main.launch
Optional Arguments:
  run_foo2 (default "0"): undocumented

roslaunch --ros-args subpose main.launch

## list dependencies
roslaunch-deps -w subpose/main.launch
Dependencies:
pubvel subpose

Missing declarations:
subpose/manifest.xml:
  <depend package="pubvel" />

## check files
rosrun roslaunch roslaunch-check main.launch

# rosparam

`rosparam` manages `ROS` parameters

## list parameters
rosparam list
/rosdistro
/rosversion
/run_id

## read specific parameter
rosparam get /run_id

## read all namespace parameters
rosparam get /

## set parameter
rosparam set /foo bbar

## delete parameter
rosparam delete /publish_message/max_vel
...

## rosparam get /publish_message/max_vel
ERROR: Parameter [/publish_message/max_vel] is not set

## save all parameters in namespace in a file (probably YAML)
rosparam dump filename namespace

## load parameters from a file (probably YAML)
rosparam load filename namespace


# tail logs
rosparam get /run_id

ls -1 ~/.ros/log/<run_id>
master.log
roslaunch-06752f9c886c-8256.log
rosout-1-stdout.log
rosout.log

