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
- **Detected Object Point Cloud Streaming**: Publishes points within bounding boxes as distinct point clouds
- **Multi-Object Detection and Localization**: Simultaneously detects and estimates positions for multiple objects in real-time
- **ROS2 Integration**: Seamless compatibility with ROS2 robotics ecosystem

## 🛠️ Installation

### 📋 Prerequisites
- **🤖 ROS2 Humble**: [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **🕵️ YOLOvX ROS**: [Setup Instructions](https://github.com/mgonzs13/yolov8_ros)
- **💻 C++ Compiler**: GCC 8 or newer
- **📚 Required Libraries**: PCL, OpenCV, and other ROS2 dependencies

### 📦 Install Dependencies
```bash
sudo apt-get update
sudo apt-get install libpcl-dev libopencv-dev
```

### 📂 Clone the Repository
```bash
cd ~/ros2_ws/src
git clone https://github.com/AbdullahGM1/ros2_depth_map_detection_localization_cpp.git
```

### 🏗️ Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select ros2_depth_map_detection_localization_cpp
source install/setup.bash
```

## 🚀 Usage

### ⚙️ Modifying the Launch File

Before running the package, modify the launch files in `ros2_depth_map_detection_localization_cpp/launch`:

1. **🖼️ Depth Map Parameters**:
```python
'width': 650, 'height': 650, 
```

2. **📏 Point Cloud Range**:
```python
'min_depth': 0.2, 'max_depth': 30.0
```

3. **🔗 Topic Names**:
```python
remappings=[
    ('/scan/points', '/change/it/to/your/topic'),  # Lidar point cloud topic
    ('/yolo/tracking', '/change/it/to/your/topic')  # YOLOv8 tracking topic
]
```

4. **🎯 YOLO Parameters**:
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
colcon build --packages-select ros2_depth_map_detection_localization_cpp
```

### 🏃 Run the Node
```bash
ros2 launch ros2_depth_map_detection_localization_cpp depth_map_detection_localization_yolo.launch.py
```

## 🤝 Contributing

Feel free to contribute to this project by creating pull requests or opening issues! 🌟 Your input is welcome and appreciated! 💡

## 🔬 Additional Notes
- Always ensure your sensor configurations match the launch file parameters
- Check ROS2 and YOLO setup before running the package
- Optimize model and detection thresholds for your specific use case
