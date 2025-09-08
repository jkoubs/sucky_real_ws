# Full Coverage Path Planning 

```bash
colcon build
source install/setup.bash
ros2 launch sucky_bringup bringup.launch.py
```

```bash

colcon build --packages-select sucky_bringup
source install/setup.bash
ros2 launch sucky_bringup depthToLaser.launch.py
```

```bash
colcon build --packages-select sucky_nav
source install/setup.bash
ros2 launch sucky_nav rtabmap_localization_v3.launch.py
```

```bash
colcon build --packages-select sucky_nav
source install/setup.bash
ros2 launch sucky_nav fcpp.launch.py
```


```bash
colcon build --packages-select sucky_nav
source install/setup.bash
ros2 launch sucky_nav fcpp_visualizers.launch.py
```

# Fox Glove
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```


# opennav_coverage 

```bash
ros2 launch sucky_nav coverage_server.launch.py 
```

```bash
ros2 launch sucky_bringup bringup.launch.py
```

```bash
ros2 launch sucky_nav rtabmap_localization_v3.launch.py
```

```bash
ros2 launch sucky_nav opennav_coverage.launch.py
```


```bash
ros2 run sucky_nav demo_coverage_optimized.py
```