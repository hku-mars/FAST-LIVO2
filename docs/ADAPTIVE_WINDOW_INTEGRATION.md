# FAST-LIVO2 自适应滑窗扩展：实现与接入指南

本文档对应 `hku-mars/FAST-LIVO2` 的 `main` 分支提交
`0d2c0346107b75b59934975adec9a6eeeb913c64`。新增代码位于
`include/adaptive_window/` 与 `src/adaptive_window/`，原始 FAST-LIVO2
仍作为实时 ESIKF 前端运行，自适应 fixed-lag 后端以模块方式并行工作。

## 1. 为什么不能直接在原代码里“改一个窗口大小”

FAST-LIVO2 不是滑窗优化器。其主调用链是：

1. `LIVMapper::processImu()`：IMU 传播和点云去畸变；
2. `VoxelMapManager::StateEstimation()`：点到平面 LIO 的迭代 Kalman 更新；
3. `VIOManager::processFrame()` / `updateState*()`：直接法视觉 Kalman 更新；
4. 两种量测按顺序写回同一个 `StatesGroup`；
5. 体素地图与视觉稀疏地图随即更新。

因此，窗口大小、关键帧和边缘化在原框架里没有对应对象。本扩展采用低侵入架构：

- FAST-LIVO2 ESIKF：高频传播、数据关联、局部地图维护和实时输出；
- adaptive-window backend：15 维导航状态 fixed-lag 优化、信息量决策、跨模态因子和外参优化；
- `FastLivoBridge`：只负责类型转换、量测导出、ROS 参数和输出，不把策略代码散落进原前端。

## 2. 模块结构

| 文件 | 职责 |
|---|---|
| `types.h` | 导航状态、外参、量测、配置和诊断数据结构 |
| `information_evaluator.*` | 从 6×6 Fisher/Gauss–Newton 信息矩阵计算 log-det、最小特征值、条件数和可靠度 |
| `adaptive_policy.*` | 模态权重、目标窗口大小、关键帧效用和边缘化候选选择 |
| `temporal_associator.*` | 原生时间戳上的最近邻跨模态配对及时间置信度 |
| `factor_builders.*` | IMU 预积分、LiDAR 点面、前端位姿、跨模态重投影和外参先验因子 |
| `fixed_lag_optimizer.*` | 线性化块求解、Huber IRLS、first-estimate Jacobian 和 Schur 边缘化 |
| `adaptive_window_backend.*` | 将策略、关联器、因子和求解器组合成统一后端 API |
| `fast_livo_bridge.*` | FAST-LIVO2/ROS 接口和反馈安全门限 |

## 3. 信息驱动的四类自适应机制

### 3.1 模态权重

LiDAR 和视觉分别导出姿态–位置块的信息矩阵

\[
\Lambda_m=J_m^{\mathsf T}W_mJ_m,\qquad m\in\{L,V\}.
\]

代码计算 `log det(Λm+εI)`、最小特征值、条件数、残差 RMSE 和内点率，得到
`reliability`。两种模态的目标权重归一化到均值约为 1，并经过一阶平滑和上下限约束。
权重作用于因子的平方根信息矩阵，而不是事后混合两条轨迹。

### 3.2 窗口大小

综合可靠度下降或出现退化时，目标窗口逐步向 `max_window_size` 增大，以保留更长时域约束；
当前信息充分或求解耗时升高时，窗口向 `min_window_size` 收缩。每次决策最多变化一帧，避免尺寸振荡。

### 3.3 自适应关键帧

关键帧效用由位姿新颖度、信息质量、时间跨度和跨模态互补性共同构成：

\[
U_k=0.30U_{\rm motion}+0.25U_{\rm info}
    +0.25U_{\rm time}+0.20U_{\rm modal}.
\]

超过阈值才建立新状态节点；时间戳非常接近的 LIO/VIO 更新会合并到同一节点。

### 3.4 自适应边缘化

当当前节点数超过目标窗口时，策略在除最新帧之外的节点中评分：高龄、高冗余、低独有信息的节点优先；
参与跨模态因子的锚点受到保护惩罚，但不会永久禁止边缘化。求解器对选定的 15 维状态执行 Schur 补：

\[
H_p=H_{rr}-H_{rm}H_{mm}^{-1}H_{mr},\qquad
b_p=b_r-H_{rm}H_{mm}^{-1}b_m.
\]

生成的先验继续约束保留状态和相机–IMU 外参。

## 4. 非硬同步跨模态约束

视觉地图点现在保留其 LiDAR 来源：`source_lidar_time_` 和
`source_lidar_point_`。当前图像通过直接法对齐后，桥接层将像素位置、源 LiDAR 点和两端原生时间戳构造成观测。
关联器在 `association_max_dt` 内选取最近的 LiDAR 状态，但不要求时间戳相等，并使用

\[
w_t=\exp\!\left(-\frac{\Delta t^2}{2\sigma_t^2}\right)
\]

连续降低非同时观测的置信度。

跨模态几何链为

\[
{}^{C}\!p=
T_{CI}\,T_{WI}^{-1}(t_c)\,T_{WI}(t_l)\,T_{IL}\,{}^{L}\!p,
\qquad
u=\pi({}^{C}\!p).
\]

代码优先使用参考 patch 与当前灰度图形成直接光度残差

\[
r_{LV,j}=\tau_c I_c(u+\delta u_j)-\tau_r I_r(\delta u_j),
\]

并通过图像梯度、投影 Jacobian 将其传递到两端状态和外参；只有 patch 不可用时才退化为重投影伪观测。
因子同时连接 LiDAR 时刻状态、图像时刻状态和 `T_CI`，因此可在线估计相机–IMU外参；
LiDAR–相机外参由 `T_CL=T_CI T_IL` 得到。投影通过 Vikit 相机模型数值求导，针孔和原项目支持的其他模型都可接入。

论文中建议使用 “timestamp-asynchronous association without hard synchronization” 或
“no exact timestamp matching requirement”，不要写成无限制的 “sync-free”：系统仍应保留有限时间门限，以防错误关联。

## 5. 原代码中的接入点

本补丁只修改了以下必要位置：

- `pointWithVar` 增加 LiDAR 来源时间；
- `VisualPoint` 增加 LiDAR 锚点、源点和源时间；
- `VIOManager::generateVisualMapPoints()` 将 LiDAR 来源传给视觉地图点；
- `LIVMapper::handleLIO()` 在 LIO 更新后导出点面因子、信息量和 IMU 数据；
- `LIVMapper::handleVIO()` 在直接法更新后导出视觉信息量和跨模态观测；
- `feedback_enabled=true` 时，只在 LIO 地图写入之前接受通过安全门限的状态反馈；外参在下一视觉帧生效。

默认 `feedback_enabled=false`。此时 `/aft_mapped_to_init` 仍是原 FAST-LIVO2 输出，
`/adaptive_window/odometry` 是独立后端结果，便于 A/B 对照和定位错误。

## 6. 编译与运行

依赖与官方项目一致；新核心只使用 Eigen，没有引入 Ceres/GTSAM。

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch fast_livo mapping_avia_adaptive.launch feedback:=false
```

确认独立后端稳定后，再测试有限反馈：

```bash
roslaunch fast_livo mapping_avia_adaptive.launch feedback:=true
```

核心算法可运行不依赖 ROS 的烟雾测试：

```bash
catkin_make run_tests_fast_livo
# 或直接运行 devel/lib/fast_livo/adaptive_window_smoke
```

## 7. 输出与诊断

- `/adaptive_window/odometry`：滑窗后端最新状态；
- `/adaptive_window/camera_imu_extrinsic`：在线估计的 `T_CI`；
- `/adaptive_window/diagnostics`：`Float64MultiArray`，依次为：
  1. 当前窗口大小；
  2. 目标窗口大小；
  3. LiDAR 权重；
  4. 视觉权重；
  5. 跨模态权重；
  6. 求解时间（ms）；
  7. 当前代价；
  8. 因子数；
  9. 累计跨模态配对数；
  10. 外参平移变化（m）；
  11. 外参旋转变化（deg）。

## 8. 推荐调试顺序

1. `enabled=true, feedback_enabled=false`，确认后端轨迹有限且无 NaN；
2. 暂时将 `max_cross_modal_factors=0`，验证仅 IMU+LIO/VIO 先验；
3. 开启跨模态因子，但用强外参先验，检查重投影残差和配对时间差；
4. 逐步放宽 `extrinsic_*_sigma`，观察外参是否收敛且跨序列一致；
5. 记录窗口大小、权重、最小特征值、ATE/RPE 和耗时，完成自适应机制消融；
6. 最后才开启状态与外参反馈，并保留现有平移/旋转安全门限。

## 9. 论文级实现前必须继续完成的三点

当前代码是可编译、可运行、方便替换因子的研究原型，但以下事项应在正式投稿实验前完成：

1. **前端事件调度完全异步化。** 官方 `sync_packages()` 在 LIVO 模式会按图像时刻切割 LiDAR 数据，
   后端关联本身不要求同时间戳，但原前端仍产生近似同刻的 LIO/VIO 更新。若论文主张完整异步输入，需将该函数改为
   “按下一事件时间处理完整 LiDAR 帧或图像帧”，并通过 IMU 将状态传播到各自原生时刻。
2. **在每次非线性迭代中重算 patch warp。** 当前已经导出原始 patch 强度残差和图像梯度，但为保持模块轻量，
   使用 first-estimate Jacobian 并冻结 FAST-LIVO2 给出的 affine/homography warp。最终版本应保存原始参考图像和法向，
   在每次后端迭代按当前两端状态与外参重新生成 warp；也可加入显式特征匹配作为独立对照因子。
3. **将数值 IMU Jacobian 换成解析预积分。** 当前 midpoint 预积分和数值 Jacobian 正确表达了状态耦合，便于原型验证；
   高频实时实验应加入 bias Jacobian、完整噪声传播和解析导数，以降低计算量并提高可复现性。

上述边界应在论文和代码 README 中如实说明；否则“无同步门槛”和“完整滑窗优化”容易被审稿人指出实现证据不足。
