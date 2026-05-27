# GICI 修改记录

## 2026-05-27

- 将 Python APE 评估升级为完整姿态矩阵计算。
- 使用 `T_B_GT` 同时修正 ground truth 位置参考点和姿态坐标系。
- 新增姿态 APE 输出，当前结果：位置 APE RMS 约 `0.0164 m`，姿态 APE RMS 约 `0.5402 deg`。
- 重新生成 `tools/visualization/output` 下的 CSV、summary 和 SVG 结果。
- 修改 RTK-RRR YAML，补齐本地数据路径、evaluation 输出和日志路径。
- 编译并运行 RTK-RRR，生成 `output/rtk_rrr_*` 结果文件。
- 使用完整姿态矩阵重新评估 RTK-RRR：位置 APE RMS 约 `0.1267 m`，姿态 APE RMS 约 `0.5206 deg`。
