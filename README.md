# FAST-LIVO2

## FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

### 1. Related video

Our accompanying video is now available on [**YouTube**](https://www.youtube.com/watch?v=aSAwVqR22mo&ab_channel=MARSLABHKU).

### 2. Related paper

[FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)

[FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)  

### 3. Codes & Datasets & Application

Our paper is currently undergoing peer review. The code, dataset, and application will be released once the paper is accepted.

### 4. Preview

This section showcases representative results of FAST-LIVO2 with high-resolution screenshots, allowing for easier observation of details.

#### 4.1 Online point cloud mapping results (Partial)

All sequences in FAST-LIVO2 private dataset are captured using low-cost Livox Avia LiDAR + pinhole camera.

<div align="center">   
    <img src="./pics/CBD_Building_03.jpg" alt="CBD_Building_03" /> 
    <p style="margin-top: 2px;">"CBD Building 03" sequence (severe LiDAR and camera degeneration)</p>
</div>
<div align="center">   
    <img src="./pics/Retail_Street.jpg" alt="Retail_Street" /> 
    <p style="margin-top: 2px;">"Retail Street" sequence</p>
</div>
<div align="center">   
    <img src="./pics/Bright_Screen_Wall.jpg" alt="Bright_Screen_Wall" /> 
    <p style="margin-top: 2px;">"Bright Screen Wall" sequence (severe LiDAR degeneration)</p>
</div>
<div align="center">
    <img src="./pics/HIT_Graffiti_Wall_01.jpg" alt="HIT_Graffiti_Wall_01" style="margin-bottom: 5px;"/>
    <img src="./pics/HIT_Graffiti_Wall_02.jpg" alt="HIT_Graffiti_Wall_02"/>
    <p style="margin-top: 2px;">"HIT Graffiti Wall" sequence (severe LiDAR degeneration)</p>
</div>
<div align="center">   
    <img src="./pics/HKU_Centennial_Garden.jpg" alt="HKU_Centennial_Garden" /> 
    <p style="margin-top: 2px;">"HKU Centennial Garden" sequence</p>
</div>
<div align="center">   
    <img src="./pics/SYSU_01.jpg" alt="SYSU_01" /> 
    <p style="margin-top: 2px;">"SYSU 01" sequence</p>
</div>
<div align="center">   
    <img src="./pics/Banner_Wall.jpg" alt="Banner_Wall" style="width: 48%;"/>   
    <img src="./pics/CBD_Building_02.jpg" alt="CBD_Building_02" style="width: 48%;"/> 
     <p style="margin-top: 2px;">Left: "Banner Wall" sequence (severe LiDAR degeneration), Right: "CBD Building 02" sequence (severe LiDAR degeneration)</p>
</div>
<div align="center">        
    <img src="./pics/HKU_Landmark.jpg" alt="HKU_Landmark" style="width: 48%;"/>        
    <img src="./pics/HKUST_Red_Sculpture.jpg" alt="HKUST_Red_Sculpture" style="width: 48%;"/>  
    <p style="margin-top: 2px;">Left: "HKU Landmark" sequence, Right: "HKUST Red Sculpture" sequence</p>
</div>
<div align="center">    
    <img src="./pics/Mining_Tunnel.jpg" alt="Mining_Tunnel_01"/>  
    <p style="margin-top: 2px;">"Mining Tunnel" sequence (severe LiDAR and camera degeneration)</p>
</div>
<div align="center">   
    <img src="./pics/HKisland01_2.jpg" alt="HKisland01_2" style="width: 48%;"/>   
    <img src="./pics/HKisland01.jpg" alt="HKisland01" style="width: 48%;"/> 
     <p style="margin-top: 2px;">"HKisland01" sequence</p>
</div>
<div align="center">
    <img src="./pics/HKairport01.jpg" alt="HKairport01" style="margin-bottom: 5px;"/>
    <img src="./pics/HKairport01_2.jpg" alt="HKairport01_2"/>
    <p style="margin-top: 2px;">"HKairport01" Sequence (LiDAR degeneration)</p>
</div>

#### 4.2 Mesh and texture reconstruction based on our dense colored point clouds

<div align="center">   
    <img src="./pics/mesh.jpg" alt="mesh" /> 
    <p style="margin-top: 2px;">(a) and (b) are the mesh and texture mapping of “CBD
 Building 01”, respectively. (c) is the texture mapping of “Retail
 Street”, with (c1) and (c2) showing local details.</p>
</div>

#### 4.3 Gaussian Splatting based on our dense colored point clouds

<div align="center">   
    <img src="./pics/nerf.jpg" alt="nerf" /> 
    <p style="margin-top: 2px;">Comparison of ground-truth image, COLMAP+3DGS, and FAST-LIVO2+3DGS in terms of render details, computational time (time for generating point clouds and estimating poses + training time), and PSNR for a random frame in “CBD
 Building 01”.</p>
</div>

#### 4.4 Fully Onboard Autonomous UAV Navigation

We mark a pioneering instance of employing a LiDAR-inertial-visual system for real-world autonomous UAV flights. Our UAV, equipped with LiDAR, a camera, and an inertial sensor, performs online state estimation (i.e., FAST-LIVO2), trajectory planning, and tracking control, all managed entirely by its onboard computer.
<div align="center">
    <img src="./pics/uav_exp1.jpg" alt="uav_exp1"/>
    <p style="margin-top: 2px;">(a) shows the overall point map of the "Basement" experiment. In (a1)-(a4), white points indicate the LiDAR scan at that moment, and colored lines depict the planned trajectory. (a1) and (a4) mark areas of LiDAR degeneration. (a2) and (a3) show obstacle avoidance. (a5) and (a6) depict the camera first-person view from indoor to outdoor, highlighting large illumination variation from sudden overexposure to normal.</p>
</div>

<div align="center">
    <img src="./pics/uav_exp2.jpg" alt="uav_exp2"/>
    <p style="margin-top: 2px;">(a) and (b) show the enlarged point maps of the "Woods" and "Narrow Opening" experiments, respectively. The red points in (a1), (a3), and (b4) represent the LiDAR scan at that moment. (a2) and (a4) represent the first-person view at the corresponding locations. (b1)-(b3) depict the third-person view.</p>
</div>

