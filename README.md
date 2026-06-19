# GICI 实验记录

## 2026-06-19：数据集 3.2 RTK/IMU TC 复现

### 本次工作汇总

| 项目 | 内容 |
| --- | --- |
| 数据集 | `data/3.2` |
| 场景 | Typical Urban，论文 Table V 中的 `3.2` |
| 配置文件 | `option/pseudo_real_time_estimation_RTK_TC_32.yaml` |
| 算法 | GICI-LIB `RTK TC`，即 RTK/IMU tightly-coupled |
| RTCM 起始日期 | `2023.03.27` |
| 回放速度 | `replay.speed: 1.0` |
| 运行方式 | `./build/gici_main option/pseudo_real_time_estimation_RTK_TC_32.yaml` |
| 运行日志 | `log/rtk_tc_32_speed1_run.log` |
| 结果覆盖 | 已覆盖 `output/rtk_tc_32_*` 与 `tools/visualization/output_32*` |

### 输出文件

| 类型 | 路径 | 说明 |
| --- | --- | --- |
| GICI NMEA 解 | `output/rtk_tc_32_solution.txt` | RTK/IMU TC 轨迹输出 |
| 原始评估 CSV | `output/rtk_tc_32_evaluation.csv` | GICI 内置 evaluation 输出 |
| 原始评估 summary | `output/rtk_tc_32_evaluation_summary.txt` | 未应用 `T_B_GT`，姿态不作为最终口径 |
| 最终 APE CSV | `tools/visualization/output_32_body_reference/ape.csv` | 应用 `T_B_GT` 后的 body-reference 评估 |
| 最终 summary | `tools/visualization/output_32_body_reference/summary.txt` | 本次建议采用的最终指标 |

### 最终评估结果

以下结果使用 `option/intrinsics_and_extrinsics.yaml` 中的 `T_B_GT`，将 `ground_truth.txt` 从 GT/Fiber-IMU 参考转换到 GICI 输出的 body reference 后统计。

| 指标 | 结果 |
| --- | ---: |
| samples | `4088` |
| utc_tow_start | `111859.976` |
| utc_tow_end | `112269.014` |
| duration | `409.038 s` |
| 3D APE mean | `0.119689 m` |
| 3D APE RMS | `0.184323 m` |
| 3D APE max_abs | `1.184841 m` |
| horizontal APE mean | `0.079648 m` |
| horizontal APE RMS | `0.134351 m` |
| horizontal APE max_abs | `1.149498 m` |
| attitude error mean | `0.348765 deg` |
| attitude error RMS | `0.386798 deg` |
| attitude error max_abs | `1.792172 deg` |
| east error RMS | `0.092069 m` |
| north error RMS | `0.097844 m` |
| up error RMS | `0.126194 m` |

### 按解状态分组

| solution_status | samples | 3D APE RMS | horizontal APE RMS |
| --- | ---: | ---: | ---: |
| fixed | `2604` | `0.102888 m` | `0.077339 m` |
| float | `1484` | `0.273891 m` | `0.198060 m` |

### 原始 evaluation 与最终口径对比

| 口径 | samples | 3D RMS | horizontal RMS | attitude RMS | 说明 |
| --- | ---: | ---: | ---: | ---: | --- |
| GICI 内置 evaluation | `4000` | `0.345915 m` | `0.332014 m` | `217.049 deg` | 直接比较原始 `ground_truth.txt`，未应用 `T_B_GT` |
| body-reference 评估 | `4088` | `0.184323 m` | `0.134351 m` | `0.386798 deg` | 应用 `T_B_GT`，本次采用 |

### 与论文 Table V 对比

论文 `doc/GICI.pdf` Table V 中，数据集 `3.2` 的 GICI-LIB `RTK TC` 结果为 `0.19 m / 1.47 deg`。

| 项目 | 论文 Table V | 本次复现 | 结论 |
| --- | ---: | ---: | --- |
| RTK TC 位置 APE | `0.19 m` | `0.184323 m` | 基本一致 |
| RTK TC 姿态 APE | `1.47 deg` | `0.386798 deg` | 口径未完全锁定；本次按 SO(3) 姿态角和 `T_B_GT` 正向定义统计 |

姿态差异的当前判断：位置复现已经对齐论文；姿态比论文小，主要可能来自论文当时使用的姿态评估脚本、外参方向、旧版标定或姿态 APE 口径不同。当前仓库未提供完整论文 Table V 的官方复现脚本。

### APE 定义

| 指标 | 计算方式 | 单位 |
| --- | --- | --- |
| east error | `solution_east - truth_east` | m |
| north error | `solution_north - truth_north` | m |
| up error | `solution_up - truth_up` | m |
| horizontal APE | `sqrt(east^2 + north^2)` | m |
| 3D APE | `sqrt(east^2 + north^2 + up^2)` | m |
| RMS | `sqrt(mean(error^2))` | 与指标一致 |

该 APE 位置定义与 `evo_ape` 的 `trans_part` 口径一致。由于 GNSS/INS 是绝对定位结果，本次未做 SE(3)/Sim(3) 轨迹对齐，只做必要的传感器参考点与坐标系转换。

### 关键代码位置

| 功能 | 文件 |
| --- | --- |
| 内置 evaluation 误差输出 | `src/stream/evaluation_formator.cpp` |
| IE ground truth 读取与插值 | `src/evaluation/evaluation_utils.cpp` |
| Python APE 与图表生成 | `tools/visualization/plot_position_error.py` |
| `T_B_GT` 外参 | `option/intrinsics_and_extrinsics.yaml` |
| IE 姿态格式转换参考 | `tools/evaluation/format_converters/src/ie_to_nmea.cpp` |

## 历史记录

### 2026-05-28：RTK-RRR 评估

#### 本次修改内容

- 将 RTK-RRR 仿实时回放速度 `replay.speed` 降低到 `0.25`，用于解决日志中的 backend pending 队列积压问题。
- 重新运行 `tools/visualization/test_plot_position_error.py`，确认评估脚本测试通过。
- 重新运行 `tools/visualization/plot_position_error.py`，基于最新 `output/rtk_rrr_evaluation.csv` 和 `option/intrinsics_and_extrinsics.yaml` 生成 RTK-RRR 误差评估。
- 更新 `tools/visualization/output_rrr` 下的 `ape.csv`、`summary.txt` 和 SVG 图表。

#### RTK-RRR 评估结果

| 指标 | 结果 |
| --- | ---: |
| aligned samples | `1594` |
| duration | `159.357 s` |
| 3D APE mean | `0.015590 m` |
| 3D APE RMS | `0.018120 m` |
| 3D APE max_abs | `0.088551 m` |
| horizontal APE RMS | `0.009987 m` |
| attitude error RMS | `0.472107 deg` |
| east error RMS | `0.007395 m` |
| north error RMS | `0.006712 m` |
| up error RMS | `0.015119 m` |

#### RTK-RRR 输出图

![2026-05-28 RTK-RRR APE Timeseries](tools/visualization/output_rrr/ape_timeseries.svg)

![2026-05-28 RTK-RRR ENU Components](tools/visualization/output_rrr/enu_components.svg)

![2026-05-28 RTK-RRR Trajectory Comparison](tools/visualization/output_rrr/trajectory_comparison.svg)

### 2026-05-24-27：RTK-TC 与 RTK-RRR 初步评估

#### 进展

- 将 Python APE 评估升级为完整姿态矩阵计算。
- 使用 `T_B_GT` 同时修正 ground truth 位置参考点和姿态坐标系。
- 新增姿态 APE 输出，并重新生成 RTK-TC 的 CSV、summary 和 SVG 结果。
- 修改 RTK-RRR YAML，补齐本地数据路径、evaluation 输出和日志路径。
- 编译并运行 RTK-RRR，生成 `output/rtk_rrr_*` 结果文件。
- 使用完整姿态矩阵重新评估 RTK-RRR，并生成 `tools/visualization/output_rrr` 下的图表。

| 模式 | 位置 APE RMS | 水平 APE RMS | 姿态 APE RMS |
| --- | ---: | ---: | ---: |
| RTK-TC | `0.0164 m` | `0.0093 m` | `0.5402 deg` |
| RTK-RRR | `0.1267 m` | `0.0687 m` | `0.5206 deg` |
