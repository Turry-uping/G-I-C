# 评估输出设计

## 目标

为 GICI 增加一条可配置的项目内评估输出路径。第一版读取 Inertial Explorer 真值文件，按时间对齐每个内部 `Solution`，计算更丰富的位置、姿态、速度、状态和协方差诊断指标，并输出 CSV 与 summary 文件，供后续绘图和可视化使用。

该功能必须通过 YAML 显式启用。现有 NMEA 解算结果输出保持不变。

## 范围

包含范围：

- 一个通用评估输出 formator/module，接入现有 stream/formator/output-tag 架构。
- 支持当前 `ground_truth.txt` 风格的 Inertial Explorer 真值解析。
- 使用真值文件中的 GPSTime 进行时间对齐。
- 输出逐历元对齐指标 CSV。
- 输出包含总体统计和 solution status 分组统计的 summary。
- 迁移可视化流程，使 Python 可以消费新的 CSV，而不是重复计算核心误差。

第一版不包含：

- CSV、TUM、NMEA 或其他真值格式。
- 实时绘图。
- 修改 estimator 行为或优化逻辑。
- 替换 NMEA 输出。

## 架构

新增一个名为 `EvaluationFormator` 的输出 formator，和 `NmeaFormator` 一样接收 `DataCluster::solution`。

该 formator 通过 YAML 配置：

```yaml
- formator:
    io: output
    tag: fmt_tc_evaluation
    type: evaluation
    truth_path: /path/to/ground_truth.txt
    output_csv: /path/to/evaluation.csv
    output_summary: /path/to/evaluation_summary.txt
    truth_format: inertial-explorer
```

estimator 通过在 `output_tags` 中增加 `fmt_tc_evaluation` 来启用：

```yaml
output_tags: [fmt_tc_solution_file, fmt_tc_evaluation]
```

主要单元：

- `EvaluationFormator`：解析 YAML 选项、加载真值、接收 solution、写入 CSV 行，并写最终 summary。
- `IeTruthReader`：解析 Inertial Explorer 真值文件，提取 GPSTime、longitude、latitude、ellipsoid height、heading、pitch 和 roll。
- `EvaluationMetrics`：插值真值，计算局部 ENU 位置误差、姿态误差、速度误差、标准差和 summary 统计。

Python 可视化脚本继续保留，但后续会优先使用 `evaluation.csv` 作为输入。

## 数据流

1. 程序启动，YAML 创建 evaluation formator。
2. formator 从 `truth_path` 加载并排序真值样本。
3. 每个 estimator 输出的 `Solution` 被传给 evaluation formator。
4. formator 通过 `Solution.coordinate` 将内部高精度 solution 位置转换为 LLA/ECEF。
5. formator 按 GPSTime 插值真值。
6. 如果对齐成功，写入一行包含 solution、truth、error、covariance 和 status 字段的 CSV。
7. 析构或程序退出时写入 summary 文件。

## CSV 字段

主 CSV 为 `evaluation.csv`。每一行对应一个成功对齐的 solution 历元。

时间：

- `timestamp_gpst`
- `truth_gpst`
- `time_offset_s`
- `elapsed_s`

Solution：

- `solution_lon_deg`
- `solution_lat_deg`
- `solution_height_m`
- `solution_roll_deg`
- `solution_pitch_deg`
- `solution_yaw_deg`

Truth：

- `truth_lon_deg`
- `truth_lat_deg`
- `truth_height_m`
- `truth_heading_deg`
- `truth_pitch_deg`
- `truth_roll_deg`

局部轨迹：

- `solution_east_m`
- `solution_north_m`
- `solution_up_m`
- `truth_east_m`
- `truth_north_m`
- `truth_up_m`

位置误差：

- `east_error_m`
- `north_error_m`
- `up_error_m`
- `horizontal_error_m`
- `translation_error_m`

姿态误差：

- `roll_error_deg`
- `pitch_error_deg`
- `yaw_error_deg`
- `attitude_error_deg`

速度误差：

- `velocity_east_error_mps`
- `velocity_north_error_mps`
- `velocity_up_error_mps`
- `velocity_horizontal_error_mps`
- `velocity_3d_error_mps`

状态诊断：

- `solution_status`
- `num_satellites`
- `differential_age_s`

来自协方差的标准差：

- `std_east_m`
- `std_north_m`
- `std_up_m`
- `std_roll_deg`
- `std_pitch_deg`
- `std_yaw_deg`
- `std_velocity_east_mps`
- `std_velocity_north_mps`
- `std_velocity_up_mps`

## 指标定义

位置误差在插值真值位置处的局部 ENU 坐标系中计算：

- `horizontal_error_m = sqrt(east_error_m^2 + north_error_m^2)`
- `translation_error_m = sqrt(east_error_m^2 + north_error_m^2 + up_error_m^2)`

真值轨迹坐标和 solution 轨迹坐标都使用第一个成功对齐的真值样本作为局部原点。

姿态误差使用最短有符号角差，并 wrap 到 `[-180, 180)`。

第一版计算：

- `roll_error_deg`
- `pitch_error_deg`
- `yaw_error_deg`
- `attitude_error_deg`

真值速度由相邻 IE 位置通过中心差分生成。边界样本使用单边差分。速度误差以局部 ENU 表示。

标准差来自 `Solution.covariance` 的对角线。位置和速度标准差保留在内部 ENU 坐标系中，姿态标准差转换为角度。

## Summary 输出

`evaluation_summary.txt` 报告：

- 对齐样本数量。
- 时间范围和持续时间。
- 跳过样本计数：
  - `skipped_before_truth`
  - `skipped_after_truth`
  - `skipped_invalid_solution`
  - `skipped_interpolation_failure`
- 每个误差列的总体统计：
  - `mean`
  - `rms`
  - `stddev`
  - `min`
  - `max`
  - `max_abs`
  - `p50`
  - `p95`
  - `p99`
- 按 `solution_status` 分组的样本数和关键 RMS 指标。

## 异常处理

启动阶段失败直接 fatal：

- `truth_path` 缺失。
- 真值文件无法打开。
- 真值格式不是 `inertial-explorer`。
- 解析出的有效真值样本少于两个。
- 输出文件无法打开。

逐 solution 失败则跳过并计数：

- solution 时间戳早于或晚于真值时间范围。
- `Solution.coordinate == nullptr`。
- solution status 无效。
- 真值插值失败。

CSV header 在第一行数据之前只写一次。如果没有产生任何对齐行，summary 仍然记录失败计数。

## 测试

增加聚焦测试或轻量测试程序，覆盖：

- 带 header 和有效数据行的 IE 真值解析。
- 跳过无效真值行。
- 按 GPSTime 插值真值。
- ENU 位置误差计算。
- 角度跨 0/360 和 -180/180 边界时的 wrap。
- 包含 RMS 和百分位数的 summary 统计。

增加一个使用当前 RTK/IMU TC 配置的集成验证：

- 在 `option/pseudo_real_time_estimation_RTK_TC.yaml` 中增加 evaluation output tag。
- 在 `data/1.1` 上运行 estimator。
- 确认 `evaluation.csv` 和 `evaluation_summary.txt` 被创建。
- 确认对齐样本数非零。
- 将位置 RMS 与当前 Python 流程对比。由于新路径使用 GPSTime 和内部高精度 `Solution`，而不是 NMEA 文本，差异是预期内的。

## 可视化迁移

保留 `tools/visualization/plot_position_error.py`，但更新为优先读取新的 `evaluation.csv`。

预期 Python 侧改动：

- 如果输入包含 `translation_error_m`，直接使用该列。
- 如果输入只有旧的 NMEA/truth 参数，保留旧路径作为 fallback。
- 使用新 CSV 字段生成现有 SVG 文件。
- 后续增加速度误差、姿态误差、状态分组，以及协方差与实际误差对比图。

更新 `tools/visualization/README.md`，将新流程作为主流程：

1. 运行启用了 evaluation formator 的 GICI。
2. 使用 `evaluation.csv` 绘图。
3. 使用 `evaluation_summary.txt` 查看总体指标。

## 分支与代码管理

开发在分支 `evaluation-output` 上进行。

第一个提交只包含本文档。实现提交应按范围拆分：

- Evaluation 解析和指标 helper。
- Formator 集成和 YAML 选项加载。
- 配置示例。
- 可视化迁移。
- 测试和文档。
