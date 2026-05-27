# GICI 实验记录

## 2026-05-27

### 今日进展

- 将 Python APE 评估升级为完整姿态矩阵计算。
- 使用 `T_B_GT` 同时修正 ground truth 位置参考点和姿态坐标系。
- 新增姿态 APE 输出，并重新生成 RTK-TC 的 CSV、summary 和 SVG 结果。
- 修改 RTK-RRR YAML，补齐本地数据路径、evaluation 输出和日志路径。
- 编译并运行 RTK-RRR，生成 `output/rtk_rrr_*` 结果文件。
- 使用完整姿态矩阵重新评估 RTK-RRR，并生成 `tools/visualization/output_rrr` 下的图表。

未解决：
-pending问题
### APE 结果

| 模式 | 位置 APE RMS | 水平 APE RMS | 姿态 APE RMS |
| --- | ---: | ---: | ---: |
| RTK-TC | `0.0164 m` | `0.0093 m` | `0.5402 deg` |
| RTK-RRR | `0.1267 m` | `0.0687 m` | `0.5206 deg` |

### RTK-TC 输出图

![RTK-TC APE Timeseries](tools/visualization/output/ape_timeseries.svg)

![RTK-TC ENU Components](tools/visualization/output/enu_components.svg)

![RTK-TC Trajectory Comparison](tools/visualization/output/trajectory_comparison.svg)

### RTK-RRR 输出图

![RTK-RRR APE Timeseries](tools/visualization/output_rrr/ape_timeseries.svg)

![RTK-RRR ENU Components](tools/visualization/output_rrr/enu_components.svg)

![RTK-RRR Trajectory Comparison](tools/visualization/output_rrr/trajectory_comparison.svg)
