# Full Coverage Path Planning 

```bash
cd ~/sucky_ws/
colcon build
source install/setup.bash
ros2 launch sucky_bringup bringup.launch.py
```

```bash
cd ~/sucky_ws/
colcon build --packages-select sucky_nav
source install/setup.bash
ros2 launch sucky_nav rtabmap_localization.launch.py
```

```bash
cd ~/sucky_ws/
colcon build --packages-select sucky_nav
source install/setup.bash
ros2 launch sucky_nav fcpp_all.launch.xml
```


# Fox Glove
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

```bash
ros2 run foxglove_bridge foxglove_bridge --ros-args -p send_buffer_limit:=100000000
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