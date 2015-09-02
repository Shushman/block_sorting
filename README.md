# Block Sorting Demo

The following rosinstall can be used to get the required dependencies for running in simulation:

```
- git: {local-name: block_sorting, uri: 'https://github.com/personalrobotics/block_sorting.git'}
- git: {local-name: herbpy, uri: 'https://github.com/personalrobotics/herbpy.git', version: feature/block_pickup}
- git: {local-name: prpy, uri: 'https://github.com/jeking04/prpy', version: feature/perception_pipeline}
- git: {local-name: herb_description, uri: 'https://github.com/personalrobotics/herb_description'}
- git: {local-name: openrave_catkin, uri: 'https://github.com/personalrobotics/openrave_catkin'}
- git: {local-name: or_parabolicsmoother, uri: 'https://github.com/personalrobotics/or_parabolicsmoother'}
- git: {local-name: or_trajopt, uri: 'https://github.com/personalrobotics/or_trajopt'}
- git: {local-name: or_cdchomp, uri: 'https://github.com/personalrobotics/or_cdchomp'}
- git: {local-name: or_urdf, uri: 'https://github.com/personalrobotics/or_urdf'}
- git: {local-name: or_ompl, uri: 'https://github.com/personalrobotics/or_ompl'}
- git: {local-name: or_sbpl, uri: 'https://github.com/personalrobotics/or_sbpl'}
- git: {local-name: comps, uri: 'https://github.com/personalrobotics/comps'}
```

To run the demo in simulation:
```
cmd> roscd block_sorting/scripts
cmd> python run_demo.py 
```
