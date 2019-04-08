# ROS Workspace, build and packages

## Workspaces

`ROS` workspace is a predefined set of directories and files where you store **related** ROS code.

You can have several workspaces but you can only work in one at any one time.

As mentioned in the repo root `README`, `Dockerfile` already sets up your default `ROS` workspace -- you are of course free to create as many as you need. Just run the same commands you can find in the `Dockerfile`:

```shell
$ docker run -it --rm ros:ros-playground /bin/bash
$ mkdir -p my_workspace/src
$ cd my_workspace/src
$ $ catkin_init_workspace
Creating symlink "/home/rosuser/my_workspace/src/CMakeLists.txt" pointing to "/opt/ros/melodic/share/catkin/cmake/toplevel.cmake"
$ cd ~/ros_ws/
$ catkin_make
```

This is a bit elaborate, but it needs to be done in order to make the setup required by `ROS` build system called `catkin`

`catkin_make` creates a few directories (besides other stuff) inside workspace -- in particular `devel` and `build` directories. You will have to source an environment file (based on the shell you use):

```shell
$ source devel/setup.bash
```

## catkin

`catkin` is a build system used in `ROS`. It's project grown and it comprises a set of `CMake` files, `bash` and `Python` scripts and whatnot. You might not interact with it into much depth if you are building simple things, but if your project is more complex you might wanna learn a bit about [CMake](https://cmake.org/).

As I said, usually, as an end user or developer you don't need to dig into it too deep -- you need to be aware of two files in order to build `ROS` programs: `CMakeLists.txt` and  `package.xml`

You will end up modifying these files to add and edit various build configuration parameters (such as library dependencies) and `catkin` will take care of the rest as you will see later on here.

## packages

`ROS` programs and/or libraries are organised into `ROS` packages. These are reusable `ROS` "units" which contain code, documentation and sometimes some data, too. They provide a way to organise your code into reusable and distributable grouped units. There are loads of `ROS` packages available on `GitHub` so have a look around.

### Creating packages

You can create new `ROS` package as follows. First change into you workspace `src` directory and then execute the commands below:
```shell
$ cd ~/ros_ws/src
$ catkin_create_pkg my_package rospy
Created file my_package/CMakeLists.txt
Created file my_package/package.xml
Created folder my_package/src
```

The first argument is the name of the package and the remaining ones are its dependencies: in this case I chose to specify only `rospy` - `ROS` python library.

This will create the following files:
```shell
$ tree my_package/
my_package/
├── CMakeLists.txt
├── package.xml
└── src
```

As mentioned earlier there are two important files: `CMakeLists.txt` and `package.xml`.

Have a look at `package.xml`: it contains some metadata about the package as well as its dependencies -- both runtime (exec) and buildtime. In our case that's only `rospy`.

Let's skip the `CMakeLists.txt` file for now as we are using `python` bindings here so we won't be building much -- we will see later on what needs to be done if using `C++` bindings (`roscpp` library).

You can place your code in the newly generated `src` directory and start it. An instance of a program running in `ROS` (or by `ROS`) is called `ROS` node. You start the `ROS` nodes by invoking `rosrun` utility (given you've built the node if it needs building).

Let's have a look how does one run `ROS` nodes.
