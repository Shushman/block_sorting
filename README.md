# Block Sorting Demo

To install run the following commands in your catkin workspace:

```
cmd> wstool init
cmd> wstool merge https://raw.githubusercontent.com/personalrobotics/pr-rosinstalls/master/herb-minimal-remote-op.rosinstall
cmd> wstool merge https://raw.githubusercontent.com/personalrobotics/block_sorting/master/rosinstall
cmd> wstool up
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
