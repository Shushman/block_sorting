# Block Sorting Demo

To install run the following commands in your catkin workspace:

```
cmd> cd src
cmd> wstool init
cmd> wstool merge https://raw.githubusercontent.com/personalrobotics/pr-rosinstalls/master/herb-minimal-remote-op.rosinstall
cmd> wstool merge https://raw.githubusercontent.com/personalrobotics/block_sorting/master/rosinstall
cmd> wstool up
cmd> cd ..
cmd> . /opt/ros/indigo/setup.bash
cmd> catkin_make
```
Note that on the second ```wstool merge``` command you will be asked if you would like to change details of herbpy and prpy. Answer ```y``` for yes. Also, if you are building on a machine other than herb0 you will need to do the following before ```catkin_make```:
```
cmd> touch src/owd/owd/CATKIN_IGNORE
cmd> touch src/owd/owd_plugins/CATKIN_IGNORE
cmd> touch src/owd/owd_plugin_example/CATKIN_IGNORE
```


To run the demo in simulation:
```
cmd> roscd block_sorting/scripts
cmd> python run_demo.py 
```

## Perception on the Real Robot
Start the `table_perception_tools` server on `herb2`:
```bash
$ rosrun tabletop_perception_tools tools_server
```
