# GICI 项目中文解读文档

本文档基于当前仓库代码整理，目标是帮助新接手项目的人快速理解：项目做什么、如何运行、运行时数据怎样流动，以及主要文件各自负责什么。

## 1. 项目定位

GICI 是一个 C++14 多传感器定位与融合项目，核心能力围绕 GNSS、IMU、Camera 的定位估计展开。它支持：

- GNSS 单点与差分定位：SPP、SDGNSS、DGNSS、RTK、PPP。
- GNSS/IMU 融合：松耦合 LC、紧耦合 TC。
- GNSS/IMU/Camera 融合：SRR、RRR 等视觉辅助融合模式。
- 文件、串口、TCP、NTRIP、V4L2、ROS 等多种数据输入输出。
- 仿实时回放、离线 post-file 处理、NMEA 输出、评估 CSV/summary 输出、可视化脚本。

核心设计是“配置驱动的节点图”。YAML 文件定义 streamer、formator、estimator 的节点和连接关系，主程序根据配置创建线程、绑定回调、启动数据流。

## 2. 构建与运行

顶层 `CMakeLists.txt` 构建一个共享库 `libgici.so`，再构建两个可执行文件：

- `gici_main`：主程序，读取 YAML 配置并运行完整数据流。
- `evaluation_tests`：评估工具测试程序。

典型构建命令见 `a.txt`：

```bash
mkdir -p build
cd build
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

典型运行命令：

```bash
./build/gici_main option/pseudo_real_time_estimation_RTK_TC.yaml
```

也可以运行当前 RTK/IMU/Camera RRR 示例：

```bash
./build/gici_main option/pseudo_real_time_estimation_RTK_RRR.yaml
```

## 3. 整体工作流

### 3.1 主程序入口

`src/gici_main.cpp` 是入口。它只接收一个参数：YAML 配置文件路径。

执行顺序：

1. 读取 YAML 配置。
2. 根据 `logging` 配置初始化 glog。
3. 注册 signal handler，便于捕获异常。
4. 创建 `NodeOptionHandle`，解析 streamer/formator/estimator 节点。
5. 创建 `NodeHandle`，实例化并绑定所有运行对象。
6. 调用 `SpinControl::run()`，进入全局运行状态。
7. 主线程以固定周期 sleep，直到 `SpinControl::ok()` 变 false。

### 3.2 YAML 节点模型

配置文件主要由三块组成：

- `stream.streamers`：数据源或数据出口，例如文件、串口、TCP、NTRIP、V4L2、ROS、post-file。
- `stream.formators`：编码/解码器，将二进制流转换为结构化数据，或将解算结果编码为输出文本/二进制。
- `estimate`：估计器节点，指定算法类型、输入角色、输出目标和算法参数。

节点 tag 有约定：

- `str_`：streamer。
- `fmt_`：formator。
- `est_`：estimator。

`input_tags` 和 `output_tags` 构成节点图。例如：

```yaml
str_gnss_rov -> fmt_gnss_rov -> est_rtk_imu_camera_rrr -> fmt_rrr_solution_file -> str_rrr_solution_file
```

### 3.3 NodeOptionHandle：解析并校验节点图

`NodeOptionHandle` 负责：

- 从 YAML 中收集 streamers、formators、estimators。
- 构建 `tag_to_node` 索引。
- 自动补齐反向连接：如果 A 的 `output_tags` 指向 B，会确保 B 的 `input_tags` 包含 A。
- 校验连接规则，例如 input formator 不能从 estimator 输入，output formator 只能连一个 estimator，普通 streamer 不能直接输出给 estimator 等。
- 为 estimator 读取每个输入 tag 对应的 roles，例如 `[rover]`、`[major]`、`[mono]`。

### 3.4 NodeHandle：实例化并绑定运行对象

`NodeHandle` 负责实际创建运行对象：

- 对普通 streamer 创建 `Streaming` 线程。
- 对 `post-file` streamer 创建 `FilesReading` 线程。
- 对 estimator 创建 `MultiSensorEstimating` 或 `MultiSensorSingleThreadEstimating`。
- 建立数据管线：
  - streamer -> formator -> estimator。
  - post-file -> estimator。
  - estimator -> formator -> streamer。
  - estimator -> estimator。
- 根据 `replay` 配置开启仿实时文件回放。
- 启动所有 streaming、files reading、estimating 线程。

### 3.5 实时/仿实时流处理

普通 `file`、`serial`、`tcp`、`ntrip`、`v4l2` streamer 通过 `Streaming` 运行。

工作方式：

1. `StreamerBase` 子类从底层流读取字节。
2. input formator 调用 `decode()` 转成 `DataCluster`。
3. `Streaming` 调用已注册的数据回调。
4. `DataIntegrationBase` 子类把 `DataCluster` 转成 estimator 使用的 `EstimatorDataCluster`。
5. estimator 收到数据后进入前端或后端处理。
6. estimator 产生 `Solution` 后，通过 output formator 编码，最后写入 output streamer。

### 3.6 post-file 离线处理

`post-file` 模式不走 `Streaming` 的字节流线程，而由 `FilesReading` 统一读取文件。

工作方式：

1. 每个 post-file streamer 创建一个 `FileReaderBase` 或其子类。
2. `FilesReading` 找到各文件初始时间。
3. 按 `step_` 推进 load timestamp。
4. 在每个时间片加载所有可用数据，按 tag 回调给 `DataIntegrationBase`。
5. 如果有 burst-load 文件，例如星历、DCB、SP3、CLK，会在开始阶段一次性加载。

post-file 会使用 `MultiSensorSingleThreadEstimating`，强制启用输入时间对齐，更适合离线复现。

### 3.7 多传感器估计线程

`EstimatingBase` 是估计线程基类。它提供：

- 独立线程循环。
- 输入回调接口 `estimatorDataCallback()`。
- 周期性调用 `process()`。
- 调用 `updateSolution()` 后输出 `DataCluster(Solution)`。

`MultiSensorEstimating` 是实时/仿实时的多线程版本：

- measurement add-in 线程：从输入缓冲取数据。
- image frontend 线程：处理图像特征。
- backend 线程：把测量送给估计器并优化。
- output 控制：按 `output_align_tag` 和 `output_downsample_rate` 输出。

`MultiSensorSingleThreadEstimating` 是 post-file 的单线程式控制版本，用于离线文件按时间顺序处理。

### 3.8 估计器后端

所有具体估计器继承或组合以下能力：

- `EstimatorBase`：维护 Ceres 图优化、状态窗口、边缘化、协方差、坐标系。
- `GnssEstimatorBase`：GNSS 原始观测建模、星历/钟差/伪距/载波/多普勒残差。
- `GnssLooseEstimatorBase`：以外部 GNSS solution 作为松耦合观测。
- `ImuEstimatorBase`：IMU 预积分、速度和 bias 状态、车辆运动约束。
- `VisualEstimatorBase`：视觉特征、重投影残差、地图点。

`Graph` 封装 Ceres problem，`ParameterBlock` 封装优化变量，`ErrorInterface` 和各种 `*_error` 封装残差。

### 3.9 结果评估和可视化

内置 C++ 评估路径：

- `EvaluationFormator` 接收 estimator 输出的 `Solution`。
- 读取 ground truth。
- 对齐时间并计算 ENU/APE/姿态误差。
- 输出 CSV 和 summary。

Python 可视化路径：

- `tools/visualization/plot_position_error.py` 可以从 NMEA + truth 重新计算误差。
- 也支持读取 evaluation CSV。
- 输出 `ape.csv`、`summary.txt`、三张 SVG 图。

## 4. 目录职责总览

| 路径 | 作用 |
| --- | --- |
| `src/gici_main.cpp` | 主程序入口。 |
| `include/gici` | 项目公共头文件，按模块划分。 |
| `src` | 项目核心实现，按模块对应 `include/gici`。 |
| `option` | 运行配置、标定文件、DCB/ATX 等辅助数据。 |
| `data` | 当前本地示例数据集，包含 GNSS/IMU/Camera/ground truth。 |
| `output` | 当前本地运行输出，例如 solution、evaluation CSV、summary。 |
| `tools` | 转换、评估、可视化、ROS 辅助工具。 |
| `ros_wrapper` | ROS 包封装，提供 ROS 主程序、消息和发布接口。 |
| `third_party` | 外部依赖源码：RTKLIB、SVO、vikit、FAST。 |
| `doc` | PDF 文档。 |
| `docs` | 当前仓库新增设计/计划/解读类文档。 |
| `build` | 本地构建产物，不是源码。 |
| `log` | 本地运行日志目录。 |

## 5. 顶层文件

| 文件 | 作用 |
| --- | --- |
| `.gitignore` | 忽略构建产物、动态库、ROS devel、数据目录、日志、第三方本地依赖等。注意其中包含 `*.atx`，所以 `option/igs14.atx` 当前属于未跟踪/本地数据类文件。 |
| `CMakeLists.txt` | 顶层 CMake。查找 Eigen/OpenCV/yaml-cpp/glog/Ceres，加入 third_party 子目录，编译 `gici` 共享库、`gici_main`、`evaluation_tests`。 |
| `LICENSE` | GPL-3.0 许可证。 |
| `README.md` | 当前不是标准项目 README，而是本地实验记录，记录 2026-05-27/28 的 RTK-TC、RTK-RRR 评估结果和图表链接。 |
| `a.txt` | 本地手工命令备忘，包含 CMake、make、运行主程序、Ceres 构建命令。 |
| `modification_summary.txt` | 当前工作区修改总结，说明 evaluation formator、RTK-TC 配置和可视化脚本的近期修改。 |

## 6. 核心源码逐文件说明

### 6.1 `src/gici_main.cpp`

主程序入口。读取 YAML，初始化日志和 signal handler，创建 `NodeOptionHandle` 与 `NodeHandle`，启动全局 spin。

### 6.2 `include/gici/utility` 与 `src/utility`

| 文件 | 作用 |
| --- | --- |
| `common.h/.cpp` | 通用数学、时间、容器辅助函数。 |
| `global_variable.h/.cpp` | 全局变量定义。 |
| `node_option_handle.h/.cpp` | YAML 节点图解析、tag 索引、连接校验、estimator 输入角色解析。 |
| `option.h/.cpp` | YAML 安全读取、字符串到枚举的转换、各类 options 加载、sensor type 判断。 |
| `rtklib_safe.h` | 安全包含 RTKLIB C 接口，处理 C/C++ 兼容。 |
| `signal_handle.h/.cpp` | 注册运行时信号处理，捕获异常或退出信号。 |
| `spin_control.h/.cpp` | 全局运行状态和周期 sleep 控制。 |
| `svo.h` | 将 SVO/vikit 类型重导出到 `gici` 命名空间，简化视觉模块依赖。 |
| `transform.h/.cpp` | 位姿、旋转、坐标变换相关工具。 |

### 6.3 `include/gici/stream` 与 `src/stream`

| 文件 | 作用 |
| --- | --- |
| `streamer.h/.cpp` | 底层流抽象。封装 serial、file、TCP server/client、NTRIP server/client、V4L2。支持 replay、文件时间标签、流同步。 |
| `streaming.h/.cpp` | 普通流线程。负责读取字节、调用 formator 解码、分发数据回调、处理日志流和输出流。 |
| `formator.h/.cpp` | 数据编码/解码抽象和主要实现。支持 RTCM2/3、GNSS raw、RINEX、image pack、IMU pack/text、NMEA、DCB、ATX、SP3、CLK、evaluation 等格式。 |
| `formator_gnss_common.cpp` | GNSS formator 公共工具，处理 RTKLIB obs/nav/sta/SSR 数据更新。 |
| `file_reader.h/.cpp` | post-file 文件读取器。用 formator 读取文件，也包含 IMU text、SP3、CLK 等特殊 reader。 |
| `files_reading.h/.cpp` | post-file 统一读取线程。按时间片读取多个文件并回调数据。 |
| `data_integration.h/.cpp` | 把 `DataCluster` 转成 `EstimatorDataCluster`。分 GNSS、IMU、Image、Solution 四类处理，并维护 GNSS 本地星历/偏差/相位中心等状态。 |
| `node_handle.h/.cpp` | 运行对象总装配器。创建 streaming、files reading、estimating，绑定所有数据流。 |
| `evaluation_formator.h/.cpp` | 输出 formator，用 solution 和 ground truth 计算逐历元评估 CSV 与 summary。 |
| `format_image.h` / `format_image.c` | 图像 pack/V4L2 格式编码解码的 C 接口。 |
| `format_imu.h` / `format_imu.c` | IMU pack 格式编码解码的 C 接口。 |

### 6.4 `include/gici/estimate` 与 `src/estimate`

| 文件 | 作用 |
| --- | --- |
| `estimating.h/.cpp` | estimator 线程基类。处理线程循环、输出回调、下采样和通用控制参数。 |
| `estimator_base.h/.cpp` | 估计器后端基类。管理 Ceres 图、状态窗口、边缘化、协方差、坐标系和状态查询。 |
| `estimator_types.h/.cpp` | 后端 ID、状态、solution、solution role、estimator type、数据包 `EstimatorDataCluster` 等核心类型。 |
| `graph.h/.cpp` | 对 Ceres problem 的封装，管理参数块、残差块、求解选项和求解调用。 |
| `parameter_block.h` | 优化变量参数块基类。 |
| `common_parameter_block.h` | 通用固定维度参数块模板，例如 position、velocity、clock、ambiguity。 |
| `pose_parameter_block.h/.cpp` | 位姿参数块，保存 SE3/四元数位姿。 |
| `speed_and_bias_parameter_block.h/.cpp` | IMU 速度和 bias 参数块。 |
| `homogeneous_point_parameter_block.h/.cpp` | 齐次地图点参数块。 |
| `pose_local_parameterization.h/.cpp` | 位姿局部参数化，用于 Ceres 在流形上更新位姿。 |
| `homogeneous_point_local_parameterization.h/.cpp` | 齐次点局部参数化。 |
| `local_parameterization_additional_interfaces.h/.cpp` | 局部参数化的附加接口。 |
| `error_interface.h/.cpp` | 残差接口和 `ErrorType` 枚举。 |
| `const_error.h` | 常量先验残差模板。 |
| `relative_const_error.h` | 相对常量约束残差模板。 |
| `relative_integration_error.h` | 相对积分约束残差。 |
| `pose_error.h/.cpp` | 位姿观测残差。 |
| `marginalization_error.h/.cpp` | 边缘化残差实现，维护线性化先验。 |
| `marginalization_error_impl.h` | 边缘化模板/内联实现。 |
| `ceres_iteration_callback.h/.cpp` | Ceres 迭代调试回调。 |
| `motion_detector.h/.cpp` | 基于 IMU/运动阈值的运动状态检测。 |

### 6.5 `include/gici/gnss` 与 `src/gnss`

| 文件 | 作用 |
| --- | --- |
| `gnss_types.h/.cpp` | GNSS 核心类型：观测、卫星、测量、解状态、通用 options、误差参数、模糊度状态。 |
| `gnss_common.h/.cpp` | GNSS 时间、观测、频率、坐标、RTKLIB 互操作等公共函数。 |
| `geodetic_coordinate.h/.cpp` | ECEF/LLA/ENU 坐标系转换和局部坐标零点管理。 |
| `gnss_parameter_blocks.h` | GNSS 常用参数块别名：position、velocity、clock、IFB、frequency、ambiguity、iono、tropo。 |
| `gnss_const_errors.h` | GNSS 常量先验残差别名。 |
| `gnss_relative_errors.h` | GNSS 相对约束残差别名。 |
| `gnss_estimator_base.h/.cpp` | GNSS 原始观测后端基类，管理卫星观测、参数块、残差、滑窗和通用优化逻辑。 |
| `gnss_estimator_base_differential.cpp` | GNSS 差分处理逻辑，例如单差/双差相关操作。 |
| `gnss_estimator_base_logger.cpp` | GNSS 中间数据日志输出。 |
| `gnss_loose_estimator_base.h/.cpp` | GNSS solution 松耦合后端基类。 |
| `spp_estimator.h/.cpp` | 单点定位 SPP。 |
| `sdgnss_estimator.h/.cpp` | 单差 GNSS 估计。 |
| `dgnss_estimator.h/.cpp` | 双差伪距 DGNSS。 |
| `rtk_estimator.h/.cpp` | RTK 载波相位差分估计。 |
| `ppp_estimator.h/.cpp` | PPP 精密单点定位。 |
| `pseudorange_error.h/.cpp` | 非差伪距残差。 |
| `pseudorange_error_sd.h/.cpp` | 单差伪距残差。 |
| `pseudorange_error_dd.h/.cpp` | 双差伪距残差。 |
| `phaserange_error.h/.cpp` | 非差载波相位残差。 |
| `phaserange_error_sd.h/.cpp` | 单差载波相位残差。 |
| `phaserange_error_dd.h/.cpp` | 双差载波相位残差。 |
| `doppler_error.h/.cpp` | 多普勒速度/频率残差。 |
| `velocity_error.h/.cpp` | GNSS 速度观测残差。 |
| `position_error.h/.cpp` | GNSS position solution 残差。 |
| `relative_isb_error.h/.cpp` | 接收机/系统间偏差相对约束残差。 |
| `ambiguity_error.h/.cpp` | 模糊度约束残差模板。 |
| `ambiguity_common.h/.cpp` | 模糊度公共工具。 |
| `ambiguity_resolution.h/.cpp` | 模糊度固定策略和参数。 |
| `ambiguity_resolution_differential.cpp` | 差分模糊度固定实现。 |
| `code_bias.h/.cpp` | 码偏差 DCB/SSR code bias 处理。 |
| `phase_bias.h/.cpp` | 相位偏差处理。 |
| `phase_center.h` | 天线相位中心 PCO/PCV 数据结构。 |
| `phase_windup.h/.cpp` | 相位 wind-up 修正。 |
| `code_phase_maps.h` | GNSS code/phase 频率映射表。 |
| `differential_measurement_align.h` | 差分测量时间/卫星对齐工具。 |

### 6.6 `include/gici/imu` 与 `src/imu`

| 文件 | 作用 |
| --- | --- |
| `imu_types.h` | IMU role、测量结构、参数结构、`SpeedAndBias` 类型。 |
| `imu_common.h/.cpp` | IMU 公共工具，例如重力、预积分辅助、噪声参数处理。 |
| `imu_estimator_base.h/.cpp` | IMU 后端基类，管理 IMU 参数、重力、body/IMU 外参、预积分残差。 |
| `imu_error.h/.cpp` | IMU 预积分残差。 |
| `speed_and_bias_error.h/.cpp` | 速度与 bias 约束残差。 |
| `hmc_error.h/.cpp` | 车辆运动/航向约束相关残差。 |
| `nhc_error.h/.cpp` | 非完整约束 NHC 残差。 |
| `roll_and_pitch_error.h/.cpp` | roll/pitch 先验或约束残差。 |
| `yaw_error.h/.cpp` | yaw 约束残差。 |

### 6.7 `include/gici/vision` 与 `src/vision`

| 文件 | 作用 |
| --- | --- |
| `image_types.h` | Camera role、相机外参估计参数、地图点等视觉类型。 |
| `feature_handler.h/.cpp` | 视觉前端总控。管理帧、特征检测跟踪、关键帧、地图点。 |
| `feature_tracker.h/.cpp` | 特征跟踪器。 |
| `feature_matcher.h/.cpp` | 特征匹配器。 |
| `visual_initialization.h/.cpp` | 视觉初始化，支持 homography/fundamental 两类初始化。 |
| `visual_estimator_base.h/.cpp` | 视觉后端基类，提供视觉特征残差、地图点和 feature handler 绑定。 |
| `reprojection_error_base.h` | 重投影残差基类。 |
| `reprojection_error.h` | 完整重投影残差。 |
| `reprojection_error_impl.h` | 重投影残差模板/内联实现。 |
| `reprojection_error_simple.h` | 简化重投影残差。 |
| `relative_pose_error.h/.cpp` | 相对位姿残差。 |
| `homogeneous_point_error.h/.cpp` | 齐次点残差。 |
| `epipolar_error.h` | 对极几何残差。 |

### 6.8 `include/gici/fusion` 与 `src/fusion`

| 文件 | 作用 |
| --- | --- |
| `multisensor_estimating.h/.cpp` | 实时/仿实时多传感器估计线程控制。负责输入缓冲、前端线程、后端线程、输出控制和 estimator 实例化。 |
| `multisensor_single_thread_estimating.h/.cpp` | post-file 离线处理的单线程估计控制。 |
| `multisensor_initializer_base.h` | 多传感器初始化器基类。 |
| `gnss_imu_initializer.h/.cpp` | GNSS/IMU 初始化，估计初始姿态、速度、bias、外参等。 |
| `gnss_imu_lc_estimator.h/.cpp` | GNSS/IMU 松耦合估计器。 |
| `spp_imu_tc_estimator.h/.cpp` | SPP/IMU 紧耦合估计器。 |
| `rtk_imu_tc_estimator.h/.cpp` | RTK/IMU 紧耦合估计器。 |
| `ppp_imu_tc_estimator.h/.cpp` | PPP/IMU 紧耦合估计器。 |
| `gnss_imu_camera_srr_estimator.h/.cpp` | GNSS/IMU/Camera 半紧耦合或 SRR 模式估计器。 |
| `spp_imu_camera_rrr_estimator.h/.cpp` | SPP/IMU/Camera RRR 估计器。 |
| `rtk_imu_camera_rrr_estimator.h/.cpp` | RTK/IMU/Camera RRR 估计器。 |

### 6.9 `include/gici/evaluation` 与 `src/evaluation`

| 文件 | 作用 |
| --- | --- |
| `evaluation_types.h` | truth sample、skip counter、统计量、CSV row 等评估数据结构。 |
| `evaluation_utils.h/.cpp` | ground truth 读取、插值、LLA 到 ENU 误差、统计量计算、summary 写出等工具。 |

## 7. 配置文件说明

| 文件 | 作用 |
| --- | --- |
| `pseudo_real_time_estimation_SPP.yaml` | 仿实时 SPP。 |
| `pseudo_real_time_estimation_SDGNSS.yaml` | 仿实时单差 GNSS。 |
| `pseudo_real_time_estimation_DGNSS.yaml` | 仿实时 DGNSS。 |
| `pseudo_real_time_estimation_RTK.yaml` | 仿实时 RTK。 |
| `pseudo_real_time_estimation_PPP.yaml` | 仿实时 PPP。 |
| `pseudo_real_time_estimation_LC.yaml` | 仿实时 GNSS/IMU 松耦合。 |
| `pseudo_real_time_estimation_SPP_TC.yaml` | 仿实时 SPP/IMU 紧耦合。 |
| `pseudo_real_time_estimation_RTK_TC.yaml` | 仿实时 RTK/IMU 紧耦合，当前本地近期评估工作主要围绕它。 |
| `pseudo_real_time_estimation_PPP_TC.yaml` | 仿实时 PPP/IMU 紧耦合。 |
| `pseudo_real_time_estimation_SRR.yaml` | 仿实时 GNSS/IMU/Camera SRR。 |
| `pseudo_real_time_estimation_SPP_RRR.yaml` | 仿实时 SPP/IMU/Camera RRR。 |
| `pseudo_real_time_estimation_RTK_RRR.yaml` | 仿实时 RTK/IMU/Camera RRR，当前路径已改成本地 `data/1.1` 和 `output/rtk_rrr_*`。 |
| `post_estimation_RTK_RRR.yaml` | 离线 post-file RTK/IMU/Camera RRR 模板，使用 `<data-directory>` 占位符。 |
| `post_estimation_RTK_RRR_rinex_imutext.yaml` | post-file RINEX + IMU text 版本的 RTK-RRR 模板。 |
| `post_estimation_PPP_rinex_sp3.yaml` | post-file PPP，使用 RINEX/SP3/CLK 精密产品。 |
| `real_time_estimation.yaml` | 实时估计配置模板。 |
| `data_storage.yaml` | 数据存储配置。 |
| `data_broadcast.yaml` | 数据广播配置。 |
| `format_conversion_and_storage.yaml` | 输入流解码后转换格式并保存。 |
| `format_conversion_and_broadcast.yaml` | 输入流格式转换后广播。 |
| `intrinsics_and_extrinsics.yaml` | 相机内参、外参等标定信息。 |
| `gici-mask.png` | 图像特征提取用 mask。 |
| `CAS0MGXRAP_20221580000_01D_01D_DCB.BSX` | DCB 码偏差产品。 |
| `igs14.atx` | 天线相位中心产品，本地存在但被 `.gitignore` 的 `*.atx` 忽略。 |

## 8. 数据、输出和文档

| 路径 | 作用 |
| --- | --- |
| `data/1.1/*.bin` | 示例二进制数据，包括 rover/reference/eph/SSR/IMU/camera 等。 |
| `data/1.1/*.bin.tag` | 文件回放时间标签，配合 RTKLIB file stream replay。 |
| `data/1.1/ground_truth.txt` | Inertial Explorer 风格真值文件。 |
| `output/rtk_tc_solution.txt` | RTK-TC NMEA 解算输出。 |
| `output/rtk_tc_evaluation.csv` | RTK-TC 内置评估逐历元 CSV。 |
| `output/rtk_tc_evaluation_summary.txt` | RTK-TC 内置评估 summary。 |
| `output/rtk_tc_evaluation_stream_sink.txt` | evaluation formator 连接的流式 sink 文件，主要结果不依赖它。 |
| `output/rtk_rrr_solution.txt` | RTK-RRR NMEA 解算输出。 |
| `output/rtk_rrr_evaluation.csv` | RTK-RRR 内置评估逐历元 CSV。 |
| `output/rtk_rrr_evaluation_summary.txt` | RTK-RRR 内置评估 summary。 |
| `output/rtk_rrr_evaluation_stream_sink.txt` | RTK-RRR evaluation sink。 |
| `doc/manual.pdf` | 原项目用户手册。 |
| `doc/GICI.pdf` | 本地加入的论文/文档 PDF，目前未被 git 跟踪。 |
| `docs/superpowers/specs/2026-05-26-evaluation-output-design.md` | evaluation 输出设计说明。 |
| `docs/superpowers/plans/2026-05-26-evaluation-output.md` | evaluation 输出实现计划。 |
| `docs/project_reading_zh.md` | 当前中文解读文档。 |

## 9. 工具目录说明

### 9.1 `tools/evaluation`

| 文件 | 作用 |
| --- | --- |
| `evaluation_tests.cpp` | C++ 评估工具测试，链接 `libgici`。 |
| `fixtures/ie_truth_sample.txt` | 测试用 Inertial Explorer truth 样本。 |
| `alignment/*` | NMEA 与 pose/position/timestamp 对齐工具。 |
| `format_converters/*` | NMEA、IE、TUM、IMU pack、IMR、image pack 等格式转换工具。 |

### 9.2 `tools/visualization`

| 文件/目录 | 作用 |
| --- | --- |
| `README.md` | APE 可视化工具说明和当前评估结论。 |
| `plot_position_error.py` | 读取 NMEA+truth 或 evaluation CSV，计算/绘制 APE、ENU 误差和轨迹 SVG。 |
| `test_plot_position_error.py` | Python 可视化脚本测试。 |
| `output/` | 默认可视化输出。 |
| `output_rrr/` | RTK-RRR 可视化输出。 |
| `output_baseline/` | baseline 可视化输出。 |
| `output_body_reference/` | body reference 可视化输出。 |
| `__pycache__/` | Python 字节码缓存，可删除。 |

### 9.3 `tools/edit_binary`

| 目录 | 作用 |
| --- | --- |
| `edit_timestamp` | 编辑/校验二进制数据时间戳，包含 cut files、correct IMU timestamp、check image timestamp、create iono parameter。 |
| `generate_replay_tag` | 生成 replay `.tag` 文件。 |
| `modify_replay_tag` | 修改 replay tag，例如整体平移时间戳。 |

### 9.4 `tools/conversions`

| 目录 | 作用 |
| --- | --- |
| `coordinate_converter` | ECEF、LLA、ENU、DMS 等坐标转换命令行小工具。 |
| `time_converter` | GPST、Unix time、epoch 等时间转换小工具。 |

### 9.5 `tools/matlab_plot`

MATLAB 绘图脚本，用于相位残差、伪距残差、电离层、模糊度、IE 误差等分析。`geoFunctions` 是 MATLAB 坐标和大气改正辅助函数。

### 9.6 `tools/ros`

ROS 辅助工具包：

- `gici_messages`：GNSS/SSR/观测消息定义。
- `gici_tools`：把 GICI 文件转换成 rosbag、发布 NMEA/TUM/IE pose、发布图像/IMU/GNSS 等。

## 10. ROS wrapper

| 路径 | 作用 |
| --- | --- |
| `ros_wrapper/src/gici/CMakeLists.txt` | ROS 包构建文件。 |
| `ros_wrapper/src/gici/package.xml` | ROS 包元信息。 |
| `ros_wrapper/src/gici/src/gici_ros_main.cpp` | ROS 版本主入口。 |
| `ros_wrapper/src/gici/src/ros_interface/*` | ROS node handle、publisher、streamer 实现。 |
| `ros_wrapper/src/gici/include/gici/ros_interface/*` | ROS 接口头文件。 |
| `ros_wrapper/src/gici/msg/*.msg` | ROS 消息定义，覆盖 GNSS observation、ephemeris、SSR、antenna position 等。 |
| `ros_wrapper/src/gici/option/*.yaml` | ROS 运行配置模板。 |
| `ros_wrapper/src/gici/rviz/gici_gic.rviz` | RViz 可视化配置。 |

## 11. third_party 外部依赖

`third_party` 是外部项目源码或本地构建产物，不建议按业务逻辑修改。

| 目录 | 作用 |
| --- | --- |
| `third_party/rtklib` | RTKLIB。提供 GNSS 原始数据解析、星历、RTCM/RINEX、PPP/RTK 辅助函数、stream 抽象等。GICI 的 GNSS 层大量复用其 C API。 |
| `third_party/rpg_svo` | SVO 视觉里程计相关源码。GICI 的视觉前端/后端复用了 frame、map、feature、reprojection、camera 等类型和算法。 |
| `third_party/rpg_vikit` | vikit 工具库，提供相机模型、数学工具、性能监控、ROS helper、Python 工具等。 |
| `third_party/fast` | FAST 角点检测器。视觉特征检测依赖它。 |

其中 `lib/*.so` 是本地构建产物，`src/`、`include/` 是上游源码。若要理解 GICI 自身逻辑，优先读 `include/gici` 与 `src`；只有调试视觉/RTKLIB 底层行为时才深入 third_party。

## 12. 当前工作区状态提示

当前 git 状态显示分支为 `evaluation-output`，且存在未跟踪文件/输出产物：

- `a.txt`
- `doc/GICI.pdf`
- `output/rtk_tc_*`
- `tools/visualization/__pycache__/`
- `tools/visualization/output_baseline/`
- `tools/visualization/output_body_reference/`

这些看起来是本地实验记录、运行输出或缓存，不应在不确认用途的情况下删除或覆盖。

## 13. 推荐阅读顺序

1. 先读 `src/gici_main.cpp`，理解入口。
2. 读 `include/gici/utility/node_option_handle.h` 与 `src/utility/node_option_handle.cpp`，理解配置如何变成节点图。
3. 读 `include/gici/stream/node_handle.h` 与 `src/stream/node_handle.cpp`，理解节点如何被实例化和绑定。
4. 读一个实际 YAML，例如 `option/pseudo_real_time_estimation_RTK_RRR.yaml`。
5. 读 `src/stream/streaming.cpp`、`src/stream/files_reading.cpp`、`src/stream/data_integration.cpp`，理解数据如何进入估计器。
6. 读 `src/fusion/multisensor_estimating.cpp`，理解实时估计线程。
7. 根据算法类型读对应 estimator，例如 RTK-RRR 读 `src/fusion/rtk_imu_camera_rrr_estimator.cpp`，再向下读 GNSS/IMU/vision base。
8. 读 `src/evaluation/evaluation_utils.cpp` 和 `src/stream/evaluation_formator.cpp`，理解评估输出。

## 14. 一句话总结

GICI 的核心不是一个固定流程，而是一个 YAML 驱动的数据流框架：streamer 负责拿到字节，formator 负责变成结构化数据，data integration 负责按传感器角色打包，estimator 负责图优化解算，output formator 再把 solution 写成文件、网络流、ROS topic 或评估结果。
