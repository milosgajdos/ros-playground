# namespace

As you work more with `ROS` and hack on more `ROS` packages you will realise that some of the names of the nodes provided either by third parties or even yourself start clashing. In order to avoid the name clashes `ROS` provides a concept of `namespaces`. Namespaces are essentially a hierarchical way of organising your `ROS` nodes so their names wont clash. Namespaces are akin to filesystem hierarchy -- you can have two files with the same name in the same path.

You can start a node into a dedicated namespace by supplying the new name via `_ns` argument:
```shell
rosrun roscpp_tutorials talker __ns:=foo chatter:=bar
```

You can verify that the `chatter` topic now exists **ONLY** within `/foo` namespace:
```shell
$ rostopic list
/foo/bar
/rosout
/rosout_agg
```

What this means is that you can launch another `talker` node in the "core" or "root" namespace and `roscore` won't command the node in `foo` namespace to exit:

```shell
rosrun roscpp_tutorials talker chatter:=bar
```

You can see that we now have two topics -- each one in separate namespace:
```shell
$ rostopic list
/bar
/foo/bar
/rosout
/rosout_agg
```

Equally, we can verify that the two nodes are now running in separate namespaces by listing the nodes:
```shell
$ rosnode list
/foo/talker
/rosout
/talker
```

Obviously, in order to be able to read the right messages you need to launch `listener` nodes into the particular namespaces.

Finally, you can also **rename** the `ROS` nodes when launching them via `__name` argument:
```shell
$ rosrun roscpp_tutorials talker __ns:=foo chatter:=bar __name:=talker_foo
```

You can see the node is now called `talker_foo`:
```
$ rosnode list
/foo/talker_foo
/rosout
```

# roslaunch

Sometimes you need to start more than one node: Robots are a collection of nodes which form a `ROS` graph. Instead of launching them laboriously one by one, there is a cli utility which you can use to do this in one command: `roslaunch`.

`roslaunch` uses *launch files* to start multiple nodes at once. Launch files allow you to specify namespaces, package remapping or whatnot just like `rosrun`. Simple launch file which would simulate what we've discussed earlier would look as follows:
```xml
<launch>
  <node name="talker" pkg="roscpp_tutorials" type="talker" output="screen" />
  <node name="listener" pkg="roscpp_tutorials" type="listener" output="screen" />
</launch>
```

The file is pretty self explanatory: it uses `XML` tags and attributes to specify how to launch particular `ROS` node. You can read more about launch files in the official [documentation](http://wiki.ros.org/roslaunch/XML).

`roslaunch` has a lot of features. It can even launch nodes on remote servers as you will see later on, but one cool thing is, when you stop it, it will stop all launched `ROS` nodes automatically. It even starts its own dedicated `roscore` process: in other words, `roslaunch` encapsulates a group of `ROS` nodes.

Place the earlier listed launch file somewhere inside the `docker` container and start the command below:
```
roslaunch sample.launch
```

You will notice all nodes being launched and both `talker` and `listener` printing their output to `stdout` as they've been instructed by `output="screen"` attribute in the launch file.

Coming back to the original example described at the beginning we can launch `talker` and `listener` into specified namespaces with remapped communication topic using the following launch file:
```xml
<launch>
  <node name="talker_foo" ns="foo" pkg="roscpp_tutorials" type="talker" output="screen">
    <remap from="chatter" to="bar"/>
  </node>
  <node name="listener" ns="foo" pkg="roscpp_tutorials" type="listener" output="screen">
    <remap from="chatter" to="bar"/>
  </node>
</launch>
```

Run the launch command as before:
```shell
roslaunch sample.launch
```

You can verify that the nodes are now running in the `foo` namespace as per the launch file and that the `talker` node has been launched as `talker_foo`:
```shell
$ rosnode list
/foo/listener
/foo/talker_foo
/rosout
```
