# GICI 修改记录

## 2026-05-27

- 将 Python APE 评估升级为完整姿态矩阵计算。
- 使用 `T_B_GT` 同时修正 ground truth 位置参考点和姿态坐标系。
- 新增姿态 APE 输出，当前结果：位置 APE RMS 约 `0.0164 m`，姿态 APE RMS 约 `0.5402 deg`。
- 重新生成 `tools/visualization/output` 下的 CSV、summary 和 SVG 结果。
