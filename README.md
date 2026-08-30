# FAST-LIVO2

## FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

### 📢 News

- 🔓 **2025-01-23**: Code released!  
- 🎉 **2024-10-01**: Accepted by **T-RO '24**!  
- 🚀 **2024-07-02**: Conditionally accepted.

### 📬 Contact

For further inquiries or assistance, please contact [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk).

## 1. Introduction

FAST-LIVO2 is an efficient and accurate LiDAR-inertial-visual fusion localization and mapping system, demonstrating significant potential for real-time 3D reconstruction and onboard robotic localization in severely degraded environments.

**Developer**: [Chunran Zheng 郑纯然](https://github.com/xuankuzcr)

<div align="center">
    <img src="pics/Framework.png" width = 100% >
</div>

### 1.1 Related video

Our accompanying video is now available on [**Bilibili**](https://www.bilibili.com/video/BV1Ezxge7EEi) and [**YouTube**](https://youtu.be/6dF2DzgbtlY).

### 1.2 Related paper

[FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)  

[FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)  

[FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)

[FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210)

### 1.3 Our hard-synchronized equipment

We open-source our handheld device, including CAD files, synchronization scheme, STM32 source code, wiring instructions, and sensor ROS driver. Access these resources at this repository: [**LIV_handhold**](https://github.com/xuankuzcr/LIV_handhold).

### 1.4 Our associate dataset: FAST-LIVO2-Dataset
Our associate dataset [**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z) used for evaluation is also available online.

### 1.5 Our LiDAR-camera calibration method
The [**FAST-Calib**](https://github.com/hku-mars/FAST-Calib) toolkit is recommended. Its output extrinsic parameters can be directly filled into the YAML file. 

### 1.6 This branch (`iridescence`)

Replaces RViz with an [Iridescence](https://koide3.github.io/iridescence/) viewer (`fastlivo_viewer`). Mapping is unchanged. `viewer:=false` is headless.

- Live registered map (`/cloud_registered`) uploaded in 50k-point chunks
- RGB colouring when the cloud has colour; XYZ heatmap (X / Y / Z) with a turbo colormap
- Heatmap range from Tukey inliers on the selected axis (outliers do not stretch the scale)
- Green trajectory (`/path`), pose axes, optional follow-track
- Orbit camera: left drag rotate, middle or right drag pan, scroll zoom
- Camera image (`/rgb_img`) in the HUD, with a scale slider
- CPU, GPU, GPU memory, host memory, and FPS as percentages
- Resizable plots of LIO / VIO timings, average residual, feature counts, and EKF path noise (position / rotation / velocity 1-sigma)
- `/fast_livo/diagnostics` published on every LIO and VIO update

## 2. Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 18.04~20.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 2.2 PCL && Eigen && OpenCV

PCL>=1.8, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=4.2, Follow [Opencv Installation](http://opencv.org/).

### 2.3 Iridescence

This branch replaces RViz with [Iridescence](https://koide3.github.io/iridescence/). `fastlivo_viewer` needs OpenGL 3.3 and an X11 display (Intel iGPU, AMD, or NVIDIA). `libglfw3` is required at runtime. The HUD GPU percentage is filled only when `nvidia-smi` is present (more support to be added later).

Ubuntu 20.04 / Noetic (source, prefix `/usr/local`):

```bash
sudo apt-get install -y libglm-dev libglfw3 libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev
git clone --recursive https://github.com/koide3/iridescence.git
cmake -S iridescence -B iridescence/build -DCMAKE_BUILD_TYPE=Release
cmake --build iridescence/build -j$(nproc)
sudo cmake --install iridescence/build
```

Custom prefix (no sudo). Replace `$HOME/.local/iridescence` if you install elsewhere:

```bash
cmake -S iridescence -B iridescence/build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local/iridescence
cmake --build iridescence/build -j$(nproc)
cmake --install iridescence/build
export CMAKE_PREFIX_PATH=$HOME/.local/iridescence:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/iridescence/lib:$LD_LIBRARY_PATH
```

Ubuntu 22.04 / 24.04:

```bash
sudo add-apt-repository -y ppa:koide3/iridescence
sudo apt install libiridescence-dev libglfw3
```

If Iridescence is not found, `catkin_make` still builds `fastlivo_mapping` and skips `fastlivo_viewer`. Launch with `viewer:=false` in that case.

### 2.4 Sophus

Sophus Installation for the non-templated/double-only version.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 2.5 Vikit

Vikit contains camera models, some math and interpolation functions that we need. Vikit is a catkin project, therefore, download it into your catkin workspace source folder.

```bash
# Different from the one used in fast-livo1
cd catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git 
```

## 3. Build

Clone this repository on the `iridescence` branch (not `hku-mars/main`):

```bash
cd ~/catkin_ws/src
git clone -b iridescence https://github.com/DethCubeHax/FAST-LIVO2.git
cd ~/catkin_ws
```

If Iridescence is in `/usr/local` or `$HOME/.local/iridescence`, CMake finds it. For any other prefix:

```bash
export CMAKE_PREFIX_PATH=/path/to/iridescence/prefix:$CMAKE_PREFIX_PATH
```

```bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Confirm the viewer binary exists:

```bash
ls ~/catkin_ws/devel/lib/fast_livo/fastlivo_viewer
```

## 4. Run

Download a bag (FAST-LIVO2-Dataset from [Global-LVBA](https://github.com/xuankuzcr/Global-LVBA) Section IV, or MARS-LVIG / NTU-VIRAL / HILTI). Replace `/path/to/sequence.bag` with the bag file.

```bash
# Avia
roslaunch fast_livo mapping_avia.launch
# MARS-LVIG Avia
# roslaunch fast_livo mapping_avia_marslvig.launch
# NTU-VIRAL Ouster
# roslaunch fast_livo mapping_ouster_ntu.launch
# HILTI Hesai
# roslaunch fast_livo mapping_hesaixt32_hilti22.launch

rosbag play /path/to/sequence.bag
```

Launches start `fastlivo_viewer` by default. Headless:

```bash
roslaunch fast_livo mapping_avia.launch viewer:=false
```

Orbit: left drag rotate, middle or right drag pan, scroll zoom. HUD: track, path, RGB / XYZ heatmap, image scale.


## 5. License

The source code of this package is released under the [**GPLv2**](http://www.gnu.org/licenses/) license. For commercial use, please contact me at <zhengcr@connect.hku.hk> and Prof. Fu Zhang at <fuzhang@hku.hk> to discuss an alternative license.
