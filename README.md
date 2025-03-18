# FAST-LIVO2 ROS2 HUMBLE

## FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

Thanks to hku mars lab chunran zheng for the open source excellent work

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
Our associate dataset [**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z) used for evaluation is also available online. **Please note that the dataset is being uploaded gradually.**

### MARS-LVIG dataset
[**MARS-LVIG dataset**](https://mars.hku.hk/dataset.html)：A multi-sensor aerial robots SLAM dataset for LiDAR-visual-inertial-GNSS fusion

## 2. Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 22.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 2.2 PCL && Eigen && OpenCV

PCL>=1.6, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=3.2, Follow [Opencv Installation](http://opencv.org/).

### 2.3 Sophus

Sophus Installation for the non-templated/double-only version.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

if build fails due to `so2.cpp:32:26: error: lvalue required as left operand of assignment`, modify the code as follows:

**so2.cpp**
```diff
namespace Sophus
{

SO2::SO2()
{
-  unit_complex_.real() = 1.;
-  unit_complex_.imag() = 0.;
+  unit_complex_.real(1.);
+  unit_complex_.imag(0.);
}
```

### 2.4 Vikit

Vikit contains camera models, some math and interpolation functions that we need. Vikit is a catkin project, therefore, download it into your catkin workspace source folder.

For well-known reasons, ROS2 does not have a direct global parameter server and a simple method to obtain the corresponding parameters. For details, please refer to https://discourse.ros.org/t/ros2-global-parameter-server-status/10114/11. I use a special way to get camera parameters in Vikit. While the method I've provided so far is quite simple and not perfect, it meets my needs. More contributions to improve `rpg_vikit` are hoped.

```bash
# Different from the one used in fast-livo1
cd fast_ws/src
git clone https://github.com/Robotic-Developer-Road/rpg_vikit.git 
```

Thanks to the following repositories for the code reference:

- [uzh-rpg/rpg_vikit](https://github.com/uzh-rpg/rpg_vikit)
- [xuankuzcr/rpg_vikit](https://github.com/xuankuzcr/rpg_vikit)
- [uavfly/vikit](https://github.com/uavfly/vikit)

### 2.5 **livox_ros_driver2**

Follow [livox_ros_driver2 Installation](https://github.com/Livox-SDK/livox_ros_driver2).

why not use `livox_ros_driver`? Because it is not compatible with ROS2 directly. actually i am not think there s any difference between [livox ros driver](https://github.com/Livox-SDK/livox_ros_driver.git) and [livox ros driver2](https://github.com/Livox-SDK/livox_ros_driver2.git) 's `CustomMsg`, the latter 's ros2 version is sufficient.

## 3. Build

Clone the repository and colcon build:

```
cd ~/fast_ws/src
git clone https://github.com/Robotic-Developer-Road/FAST-LIVO2.git
cd ../
colcon build --symlink-install --continue-on-error
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
ros2 launch fast_livo mapping_aviz.launch.py use_rviz:=True
ros2 bag play -p Retail_Street  # space bar controls play/pause
```

## 5. License

The source code of this package is released under the [**GPLv2**](http://www.gnu.org/licenses/) license. For commercial use, please contact me at <zhengcr@connect.hku.hk> and Prof. Fu Zhang at <fuzhang@hku.hk> to discuss an alternative license.