# Le_Turtelbot_VENTOUZZZ

## Start
Run this to create pixi env with ros2 and source the venv :
```
pixi shell
```

Then launch navigation stack
```
ros2 launch nav2_bringup bringup_launch.py map:=$(pwd)/maps/my_map.yaml params_file:=$(pwd)/params/nav2_params.yaml
```

Finaly run custom node
```
ros2 run waypoint_ventouzzz waypoint 
```