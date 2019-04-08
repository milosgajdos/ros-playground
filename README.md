# ros-playground

This repo contains a collection of [ROS](http://wiki.ros.org/) samples written in `C++` and `Python` using either or both of `roscpp` and `rospy` libraries.

## Environment setup

All the examples in this repo have been run in [Docker](https://www.docker.com/) containers built off the [official ROS images](https://hub.docker.com/r/library/ros/). As for the particular ROS release, the examples have been tested on [Melodic Morenia](http://wiki.ros.org/melodic) release.

### Docker setup

Pull down all necessary `docker` images:

```
docker pull ros                                # barebone ROS install
docker pull ros:melodic-ros-core               # same as ros:latest
docker pull ros:melodic-ros-base               # basic tools and libraries
docker pull ros:melodic-robot-stretch          # basic install for robots
docker pull ros:melodic-perception-stretch     # basic install for perception tasks
```

Build a new `docker` image using the `Dockerfile` in the root directory of this repo. Note that the root directory of this repo contains `.dockerignore` file which excludes all the local files from the `docker` build context:

```
docker build --tag ros:ros-playground -f Dockerfile .
```

The newly built `ros:ros-playground` image will have all the ROS system dependencies installed along with a skeleton [catkin](http://wiki.ros.org/catkin/Tutorials) build workspace initialized in `~rosuser/ros_ws/src` directory.

Test the newly built image by checking the ROS version:

```
$ docker run -it --rm ros:ros-playground rosversion -d
melodic
```
If everything went fine, you should now be ready to start hacking on `ROS`.

**NOTE:** `ros:ros-playground` image `Dockerfile` creates `rosuser` with `sudo` privileges and its own dedicated [ROS workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in `$HOME/ros_ws` directory. You will do most of the work inside the workspace. Furthermore, the `.bashrc` script `source`s ROS environment setup so you don't have to do it manually.

`ROS` uses `~/.ros/` for various things like storing logs and debugging info, so if you want to persist this directory you'll have to mount it in as a volume into `rosuser` home directory:

```
$ docker run -it --rm -v "$PWD/.ros/:/home/rosuser/.ros" ros:ros-playground ls -la
drwxr-xr-x 1 rosuser rosuser 4096 Apr  2 00:38 .
drwxr-xr-x 1 root    root    4096 Apr  2 00:33 ..
-rw-r--r-- 1 rosuser rosuser  220 Apr  4  2018 .bash_logout
-rw-r--r-- 1 rosuser rosuser 3771 Apr  4  2018 .bashrc
-rw-r--r-- 1 rosuser rosuser  807 Apr  4  2018 .profile
drwxr-xr-x 2 rosuser rosuser   64 Apr  7 21:17 .ros
drwxr-xr-x 5 rosuser rosuser 4096 Apr  2 00:38 ros_ws
```
