# colour_detector

ROS 2 Humble package for HSV colour detection from the PUSANTARA camera topic.

## Input
- `/camera/image_raw` (`sensor_msgs/msg/Image`)

## Outputs
- `/colour/detections` (`std_msgs/msg/String`, JSON)
- `/colour/annotated` (`sensor_msgs/msg/Image`)

Coordinates are image pixels measured from the top-left of the frame. Each detection contains the contour centroid (`center_x`, `center_y`), bounding rectangle (`x`, `y`, `width`, `height`), area and normalized centre coordinates. These are not 3D/world coordinates.

## Build
```bash
cd ~/vectra_ws
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-opencv python3-numpy ros-humble-cv-bridge
colcon build --packages-select colour_detector --symlink-install
source install/setup.bash
```

## Run
Make sure the existing `webcam_publisher` is publishing `/camera/image_raw`. Do not start a second camera publisher if it is already running.

```bash
source /opt/ros/humble/setup.bash
source ~/vectra_ws/install/setup.bash
ros2 run colour_detector colour_detector --ros-args -p target_colour:=red -p show_ui:=true
```

Supported target colours: `red`, `green`, `blue`, `yellow`, `orange`, `purple`.

## Parameters
- `image_topic`: `/camera/image_raw`
- `detections_topic`: `/colour/detections`
- `annotated_topic`: `/colour/annotated`
- `target_colour`: `red` by default
- `min_area`: `500.0` pixels² by default; increase to reject small noise
- `kernel_size`: `5` by default
- `show_ui`: `true` by default; set false for headless operation

Example for blue, without a GUI:
```bash
ros2 run colour_detector colour_detector --ros-args -p target_colour:=blue -p show_ui:=false
```

Inspect detections:
```bash
ros2 topic echo /colour/detections
```

Inspect the annotated image topic:
```bash
ros2 topic info /colour/annotated
```
