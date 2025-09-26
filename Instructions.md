# Full Coverage Path Planning 

```bash
colcon build
source install/setup.bash
ros2 launch sucky_bringup bringup.launch.py
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



### Collect RTAB-Map Input Topics [REAL]

```bash
ssh_sucky
ros2 launch sucky_bringup bringup.launch.py
ros2 bag record \
  --compression-mode file \
  --compression-format zstd \
  -o ~/bags/rosbag2_$(date +%Y_%m_%d-%H_%M_%S) \
  /diffbot_base_controller/odom \
  /camera/d455/color/camera_info \
  /camera/d455/color/image_raw/compressed \
  /camera/d455/depth/image_rect_raw/compressedDepth \
  /scan \
  /tf \
  /tf_static
```


### Foxglove

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
foxglove-studio
```

