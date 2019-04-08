# roscore

`ROS` has a distributed architecture at the core of which is a daemon called `roscore`. This daemon fulfills a few tasks among which the most important one is `ROS` node discovery i.e. it provides a way `ROS` nodes (instances of `ROS` packages) communicate with each other.

No matter what you are trying to do with `ROS` this daemon must be running:
```shell
$ roscore
... logging to /home/rosuser/.ros/log/ba80c77a-5984-11e9-9558-0242ac110002/roslaunch-872941295ac5-595.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://872941295ac5:34833/
ros_comm version 1.14.3


SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.3

NODES

auto-starting new master
process[master]: started with pid [605]
ROS_MASTER_URI=http://872941295ac5:11311/

setting /run_id to ba80c77a-5984-11e9-9558-0242ac110002
process[rosout-1]: started with pid [616]
```

Once it's up and running you start `ROS` node(s). `ROS` nodes are instances of `ROS` programs which are part of some `ROS` package. `ROS` nodes immediately connect to `roscore` to "register" themselves with it so that the other `ROS` nodes can discover them. `roscore` is an enabler of the communication between `ROS` nodes i.e. sort of like a [service discovery](https://en.wikipedia.org/wiki/Service_discovery) daemon.

`ROS` nodes communicate directly to each other -- as mentioned earlier `roscore` merely enabled `ROS` node discovery. It actually does a bit more, but for now just remember `roscore` must be running before you attempt to start any `ROS` node.

# rosrun

You can start arbitrary `ROS` program using `rosrun` utility by running the following command:
```shell
$ rosrun PACKAGE_NAME EXECUTABLE [OPTIONAL_ARGUMENTS]
```

You must specify what package you want to run a particular executable (runnable `ROS` program) from.

Let's have a look at some examples. The image we built comes with a few packages installed which ship various `ROS` packages. You can find them in `/opt/ros/melodic/share/` directory. We'll play with `roscpp_tutorials` package.

We will start `talker` and `listener` `ROS` nodes to demonstrate the usage of `rosrun` command.

First, make sure your `roscore` daemon is running. We'll `exec` into the same container and start a talker:
```shell
docker exec -it <container_id_or_name> /bin/bash
$ rosrun roscpp_tutorials talker
[ INFO] [1554677243.551496400]: hello world 0
[ INFO] [1554677243.652958900]: hello world 1
[ INFO] [1554677243.756993900]: hello world 2
[ INFO] [1554677243.851780600]: hello world 3
[ INFO] [1554677243.953524500]: hello world 4
```

As you can see the `talker` is now running and printing messages to `stdout`. `talker` actually does more than that: it sends the messages to a `topic` where they can be read by a topic subscriber. We will talk more about the topics later on but for now just keep in mind that `talker` sends (publishes) messages that can be read by another `ROS` node (subscriber).

We will use a node called `listener` to read and accept these messages. Once again we will `exec` into the same container and start the `listener` node:
```shell
docker exec -it <container_id_or_name> /bin/bash
rosrun roscpp_tutorials listener
[ INFO] [1554677432.502593500]: I heard: [hello world 16]
[ INFO] [1554677432.603361100]: I heard: [hello world 17]
[ INFO] [1554677432.704055800]: I heard: [hello world 18]
[ INFO] [1554677432.802685400]: I heard: [hello world 19]
[ INFO] [1554677432.904908300]: I heard: [hello world 20]
[ INFO] [1554677433.004969700]: I heard: [hello world 21]
```

`listener` receieves the messages and prints them to its `stdout`.

## topics

Quick note on topics before we dive in more into them later on. `topic`s provide a way to communicate between two `ROS` nodes asynchronously. It's essentiallt a `ROS` pub/sub with a twist that the messages are delivered between the nodes directly i.e. peer-to-peer.

When a *publisher* node starts it registers with `roscore` and tells it it wants to send messages to particular topic. `roscore` "makes a note" of it and off it goes sending the messages.

When a subscriber starts it can register its interest with `roscore` in particular topics. `roscore` then tells it what nodes are sending the message to those particular topics and subscriber then connects to the publisher node and after a handshake they start communicating directly.

This is nicely illustrated in the official [documentation](http://wiki.ros.org/ROS/Technical%20Overview#Example)

So what topics do these two `ROS` nodes communicate on? We can find out by running the `rostopi` command to liste all existing topics:
```shell
$ rostopic list
/chatter
/rosout
/rosout_agg
```

Ignore for now `/rosout` and `/rosout_agg` which are created automatically by `roscore` and you'll see that the topics name `talker` and `listener` communicate over is called `/chatter`.

You can also list all running nodes using `rosnode` utility:
```shell
$ rosnode list
/listener
/rosout
/talker
```
**NOTE:** `ROS` node names must be unique. If a node with the same name as already running node is launched i.e. the same node is launched twice, `roscore` tells the earlier launched node to exit to make way for the new node. Try to run another intance of `roscpp_tutorials listener` node and you will see that the first node would exit:
```shell
...
[ WARN] [1554678756.058634600]: Shutdown request received.
[ WARN] [1554678756.059243900]: Reason given for shutdown: [new node registered with same name]
```

### Remapping

You can **remap** existing topics to different name by specifying the remap as an argument when launching the node:
```shell
rosrun roscpp_tutorials talker chatter:=foobar
```

You will notice that `/chatter` topic has been renamed to `/foobar`:
```shell
$ rostopic list
/foobar
/rosout
/rosout_agg
```

This will come in handy later on.
