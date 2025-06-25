# FAST-LIVO2 ROS2 HUMBLE

## FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

### :star: Acknowledgments

This project extends the following open-source contributions:

- **Original work**: [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) by Chunran Zheng (HKU MARS Lab).
- **ROS2 Humble port**: [FAST-LIVO2/tree/humble](https://github.com/Robotic-Developer-Road/FAST-LIVO2/tree/humble) by Robotic-Developer-Road.

### 📢 News

- 🔓 **2025-01-23**: Code released!  
- 🎉 **2024-10-01**: Accepted by **T-RO '24**!  
- 🚀 **2024-07-02**: Conditionally accepted.

### 📬 Contact

If you have any questions, please feel free to contact: Chunran Zheng [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk).

## 1. Introduction

FAST-LIVO2 is an efficient and accurate LiDAR-inertial-visual fusion localization and mapping system, demonstrating significant potential for real-time 3D reconstruction and onboard robotic localization in severely degraded environments.

<div align="center">
    <img src="pics/Framework.png" width = 100% >
</div>

### 1.1 Related video

Our accompanying video is now available on [**Bilibili**](https://www.bilibili.com/video/BV1Ezxge7EEi) and [**YouTube**](https://youtu.be/6dF2DzgbtlY).

### 1.2 Related paper

[FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)  

[FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)

### 1.3 Our hard-synchronized equipment

We open-source our handheld device, including CAD files, synchronization scheme, STM32 source code, wiring instructions, and sensor ROS driver. Access these resources at this repository: [**LIV_handhold**](https://github.com/xuankuzcr/LIV_handhold).

### 1.4 Our associate dataset: FAST-LIVO2-Dataset
Our associate dataset [**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z) used for evaluation is also available online.

### MARS-LVIG dataset
[**MARS-LVIG dataset**](https://mars.hku.hk/dataset.html)：A multi-sensor aerial robots SLAM dataset for LiDAR-visual-inertial-GNSS fusion.

### 1.5 Our LiDAR-camera calibration method
The [**FAST-Calib**](https://github.com/hku-mars/FAST-Calib) toolkit is recommended. Its output extrinsic parameters can be directly filled into the YAML file.

## 2. Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 22.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 2.2 PCL && Eigen && OpenCV

PCL>=1.8, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=4.2, Follow [Opencv Installation](http://opencv.org/).

### 2.3 Sophus

#### Binary installation
```bash
cd ~
git clone https://github.com/strasdat/Sophus.git -b 1.22.10
cd Sophus
mkdir build && cd build && cmake ..
make
sudo make install
```

### 2.4 Vikit

Vikit provides essential camera models, math utilities, and interpolation functions. As an ament package for ROS2, download its source into your colcon workspace's src folder. Additionally, I've added OpenCV fisheye distortion correction to the equidistant camera model in vikit_common.

```bash
# Different from the one used in fast-livo1
cd ~
git clone https://github.com/Rhymer-Lcy/rpg_vikit_ros2_fisheye.git
mkdir -p ~/fast/src/
cp -r ./rpg_vikit_ros2_fisheye/{vikit_common,vikit_ros} ~/fast_ws/src/
```

Thanks to the following repositories for the code reference:

- [xuankuzcr/rpg_vikit](https://github.com/xuankuzcr/rpg_vikit)
- [integralrobotics/rpg_vikit](https://github.com/integralrobotics/rpg_vikit)

### 2.5 **livox_ros_driver2**

Follow [livox_ros_driver2 Installation](https://github.com/Livox-SDK/livox_ros_driver2).

```bash
cd ~/fast_ws/src/
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

why not use `livox_ros_driver`? Because it is not compatible with ROS2 directly. actually i am not think there s any difference between [livox ros driver](https://github.com/Livox-SDK/livox_ros_driver.git) and [livox ros driver2](https://github.com/Livox-SDK/livox_ros_driver2.git) 's `CustomMsg`, the latter 's ros2 version is sufficient.

## 3. Build

Clone the repository and colcon build:

```
git clone https://github.com/Rhymer-Lcy/FAST-LIVO2-ROS2-MID360-Fisheye.git
cd livox_ros_driver2
./build.sh humble
source ~/fast_ws/install/setup.bash
```

## 4. Run our examples

Download our collected rosbag files via OneDrive ([**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z)). 

### convert rosbag

convert ROS1 rosbag to ROS2 rosbag
```bash
pip install rosbags
rosbags-convert --src Retail_Street.bag --dst Retail_Street
```
- [gitlab rosbags](https://gitlab.com/ternaris/rosbags)
- [pypi rosbags](https://pypi.org/project/rosbags/)

### change the msg type on rosbag

Such as dataset `Retail_Street.db3`, because we use `livox_ros2_driver2`'s `CustomMsg`, we need to change the msg type in the rosbag file. 
1. use `rosbags-convert` to convert rosbag from ROS1 to ROS2.
2. change the msg type of msg type in **metadata.yaml** as follows:

**metadata.yaml**
```diff
rosbag2_bagfile_information:
  compression_format: ''
  compression_mode: ''
  custom_data: {}
  duration:
    nanoseconds: 135470252209
  files:
  - duration:
      nanoseconds: 135470252209
    message_count: 30157
    path: Retail_Street.db3
    ..............
    topic_metadata:
      name: /livox/lidar
      offered_qos_profiles: ''
      serialization_format: cdr
-     type: livox_ros_driver/msg/CustomMsg
+     type: livox_ros_driver2/msg/CustomMsg
      type_description_hash: RIHS01_94041b4794f52c1d81def2989107fc898a62dacb7a39d5dbe80d4b55e538bf6d
    ...............
.....
```

### Run the demo

Do not forget to `source` your ROS2 workspace before running the following command.

```bash
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True use_sim_time:=True
ros2 bag play -p Retail_Street  # Use space bar to play/pause
```

```bash
ros2 launch fast_livo mapping_aviz_metacamedu.launch.py use_rviz:=True use_sim_time:=True  # Configuration for MID360-Fisheye dataset
ros2 bag play -p $BAG_PATH  # Use space bar to play/pause (self-collected MID360-Fisheye dataset)
```

## 5. License

The source code of this package is released under the [**GPLv2**](http://www.gnu.org/licenses/) license. For commercial use, please contact me at <zhengcr@connect.hku.hk> and Prof. Fu Zhang at <fuzhang@hku.hk> to discuss an alternative license.