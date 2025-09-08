# Full Coverage Path Planning 

```bash
colcon build
source install/setup.bash
ros2 launch sucky_bringup bringup.launch.py
ros2 launch sucky_nav fcpp.launch.py
ros2 launch sucky_nav fcpp_visualizers.launch.py
```


```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```
