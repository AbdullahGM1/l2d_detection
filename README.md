# 🤖 ROS2 Depth Map Detection and Localization Package

## 📝 Description

This ROS2 package provides a robust solution for converting point cloud data into depth maps, integrating advanced detection capabilities with YOLO to detect objects within the depth map. The package generates two types of depth maps: one representing the original scene and another highlighting the depths of detected objects. Additionally, it publishes pose data (x,y,z) for detected objects, aiding in accurate object localization in 3D space.

## 📋 Table of Contents
1. [Demonstration](#demonstration)
2. [Features](#features)
3. [Installation](#installation)
4. [Usage](#usage)
5. [Node](#node)
6. [Contributing](#contributing)

## 🎥 Demonstration

### 📸 Depth Map Detection and Localization in Action

<p align="center">
  <img src="images/1.png" alt="Depth Map Detection 1" width="500"/>
  <img src="images/2.png" alt="Depth Map representing highlighting the depths of detected objects 2" width="500"/>
</p>

<p align="center">
  <img src="images/3.gif" alt="Depth Map representing highlighting the depths of detected objects and pose gif" width="500"/>
</p>

## ✨ Features

- **Original Depth Map Generation**: Converts the lidar point clouds to depth maps representing the entire scene
- **Object-specific Depth Map**: Isolates and enhances depth data specifically for detected objects, based on YOLOv8 outputs
- **3D Position Estimation**: Calculates and publishes the average pose of detected objects based on depth data
- **Multi-Object Detection and Localization**: Simultaneously detects and estimates positions for multiple objects in real-time
- **360-Degree Depth Mapping**: Generates dual depth maps for both positive (+) and negative (-) z-axis orientations, enabling comprehensive 360-degree environmental coverage and object detection capabilities

- **ROS2 Integration**: Seamless compatibility with ROS2 robotics ecosystem

## 🛠️ Installation

### 📋 Prerequisites
- **🤖 ROS2 Humble**: [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **🕵️ YOLO ROS**: [Setup Instructions](https://github.com/AbdullahGM1/yolo_ros)
- **💻 C++ Compiler**: GCC 8 or newer
- **📚 Required Libraries**: PCL, OpenCV, and other ROS2 dependencies

### 📦 Install Dependencies
```bash
sudo apt-get update
sudo apt-get install libpcl-dev libopencv-dev
sudo apt-get update && sudo apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-message-filters
```

### 📂 Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/AbdullahGM1/l2d_detection.git
```

## 🚀 Usage

### ⚙️ Modifying the Launch File

Before running the package, modify the launch files in `depth_detection.launch.py/launch`:

1. **🖼️ Depth Map Parameters**:
```python
'width': 650, 'height': 650
```
2. **📏 Depth Map Scale Vector**
```python
'ScaleVector': 4.0
```
3. **📏 Point Cloud Range**:
```python
'MinDepth': 0.2, 'MaxDepth': 30.0
```
4. **🔗 Topic Names**:
```python
remappings=[
    ('/scan/points', '/change/it/to/your/topic'),  # Lidar point cloud topic
    ('/yolo/tracking', '/change/it/to/your/topic')  # YOLOv11 tracking topic
]
```
5. **🎯 YOLO Parameters**:
```python
launch_arguments={
    'model': '/path/to/model.pt',
    'threshold': '0.5',
    'input_image_topic': '/depth_map', 
    'device': 'cuda:0'
}.items()
```

### 🔨 Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select l2d_detection
source install/setup.bash
```

### Run the Launch File
```bash
ros2 launch l2d_detection depth_detection.launch.py
```

> ### ⚠️ Important Notes
* If you want to run the package with simulation, you need to follow the steps in the following repo [SMART-Track-sim-setup.](https://github.com/AbdullahGM1/SMART-Track-sim-setup./tree/main)
* If you want to run just the `lidar_camera_fusion_with_detection.cpp` node without the simulation, you need to comment these parts in the launch file:
```py
    # ld.add_action(gz_launch)
    # ld.add_action(map2pose_tf_node)
    # ld.add_action(cam_tf_node)
    # ld.add_action(lidar_tf_node)
    # ld.add_action(ros_gz_bridge)
    # ld.add_action(mavros_launch)
    # ld.add_action(rviz_node)
```

## 🔌 Node Details

### Subscribed Topics
| Topic Name | Message Type | Description | Rate |
|------------|--------------|-------------|------|
| `/scan/points` | `sensor_msgs/PointCloud2` | Input point cloud data from LiDAR sensor | 10-30 Hz |
| `/yolo/tracking` | `vision_msgs/Detection2DArray` | YOLO detection and tracking results | 10-30 Hz |

### Published Topics
| Topic Name | Message Type | Description | Rate |
|------------|--------------|-------------|------|
| `/depth_map` | `sensor_msgs/Image` | Original depth map generated from point cloud | 10-30 Hz |
| `/depth_map_objects` | `sensor_msgs/Image` | Depth map highlighting detected objects | 10-30 Hz |
| `/object_poses` | `geometry_msgs/PoseArray` | 3D positions of detected objects | 10-30 Hz |

### Parameters
| Parameter Name | Type | Default | Description |
|----------------|------|---------|-------------|
| `width` | Integer | 650 | Width of generated depth map |
| `height` | Integer | 650 | Height of generated depth map |
| `ScaleVector` | Float | 4.0 | The Scale Vector of generated depth map |
| `min_depth` | Float | 0.2 | Minimum depth range (meters) |
| `max_depth` | Float | 30.0 | Maximum depth range (meters) |
| `model_path` | String | "" | Path to YOLO model file |
| `threshold` | Float | 0.5 | Detection confidence threshold |
| `device` | String | "cuda:0" | Computing device (GPU/CPU) |


## 🤝 Contributing

Feel free to contribute to this project by creating pull requests or opening issues! 🌟 Your input is welcome and appreciated! 💡

## 🔬 Additional Notes
- Always ensure your sensor configurations match the launch file parameters
- Check ROS2 and YOLO setup before running the package
- Optimize model and detection thresholds for your specific use case

