# Le_Turtelbot_VENTOUZZZ

## Start
Run this to create pixi env with ros2
```
pixi init . -c robostack-jazzy -c conda-forge
pixi add ros-jazzy-desktop
```

Then add this to pixi.toml
```
[activation.env]
ROS_DOMAIN_ID = "1"
```