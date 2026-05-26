# GICI 误差可视化与 APE 验证

本目录用于管理 GICI 输出结果与参考真值之间的误差计算和可视化。当前流程以论文中的 APE（Absolute Pose Error，绝对位姿误差）作为首选误差指标。

## 运行命令

在仓库根目录运行：

```bash
python3 tools/visualization/plot_position_error.py \
  --solution output/rtk_tc_solution.txt \
  --truth data/1.1/ground_truth.txt \
  --output-dir tools/visualization/output
```

脚本只依赖 Python 标准库，不需要 `matplotlib` 也能生成 SVG 图。

## 输出文件

`tools/visualization/output` 中只保留当前 APE 工作流需要的文件：

- `ape.csv`：时间对齐后的样本、局部 ENU 轨迹坐标、APE 和 ENU 误差分量。
- `ape_timeseries.svg`：APE 随时间变化图，包含 3D APE、水平 APE 和 Up 方向误差。
- `trajectory_comparison.svg`：GICI 轨迹与 ground truth 轨迹在本地 EN 平面中的对比图。
- `enu_components.svg`：East、North、Up 三个方向的有符号误差分量。
- `summary.txt`：样本数量、时间范围、指标含义和统计结果。

## 误差指标解释

- `ape_translation_m`：首选 APE 指标，三维位置误差范数，计算方式为 `sqrt(E^2 + N^2 + U^2)`，单位为米。
- `ape_horizontal_m`：水平面 APE，只使用 East 和 North，计算方式为 `sqrt(E^2 + N^2)`，适合观察平面轨迹形状误差。
- `east_error_m`：East 方向有符号误差。正值表示 GICI 结果相对真值偏东。
- `north_error_m`：North 方向有符号误差。正值表示 GICI 结果相对真值偏北。
- `up_error_m`：Up 方向有符号误差。正值表示 GICI 结果相对真值偏高。
- `mean`：算术平均值。对有符号 ENU 分量来说，正负误差可能互相抵消。
- `rms`：均方根误差，是当前最建议关注的总体精度统计量。
- `max_abs`：对有符号分量表示最大绝对误差；对 APE 这种非负量表示最大误差。

## 当前结果验证

论文 `doc/GICI.pdf` 的表 V 写明：数据集 `1.1` 中 GICI-LIB `RTK TC` 的 APE 结果为 `0.03 m / 0.55 deg`，其中前者是位置 APE，后者是姿态 APE。

当前使用 `output/rtk_tc_solution.txt` 与 `data/1.1/ground_truth.txt` 计算得到：

```text
ape_translation_m RMS: 0.328985 m
ape_horizontal_m  RMS: 0.320537 m
east_error_m      RMS: 0.278789 m
north_error_m     RMS: 0.158179 m
up_error_m        RMS: 0.074074 m
```

因此，当前 NMEA 输出文件的 3D 位置 APE RMS 约为 `0.329 m`，没有达到论文表 V 中 `1.1 / RTK TC` 的 `0.03 m` 位置 APE。这个结果说明当前可视化流程和误差计算已经能正常工作，但当前输入结果文件还不能视为论文表 V 的完全复现结果。

可能原因：

- 当前脚本使用的是 NMEA 输出中的 `GGA` 经纬高，NMEA 文本输出存在格式精度限制。
- 当前比较的是 `rtk_tc_solution.txt` 中的位置字段，论文表 V 的 APE 可能来自更高精度的内部结果或论文评估流程。
- ground truth 文件头中包含 IMU 到 GNSS 天线杆臂信息，而当前脚本只比较位置坐标，没有额外做杆臂或位姿中心转换。
- 当前配置是本地修改后的 `option/pseudo_real_time_estimation_RTK_TC.yaml`，需要确认其参数与论文实验配置完全一致。

## MATLAB 与 VSCode

如果需要在 VSCode 中运行 `.m` 文件，建议安装 MathWorks 官方扩展 `MATLAB`，扩展 ID 是 `MathWorks.language-matlab`。

基本步骤：

1. 本机安装 MATLAB，或使用支持的 MATLAB Online。
2. 在 VSCode 扩展市场安装 `MATLAB` 扩展。
3. 打开 `.m` 文件。
4. 使用文件顶部的 Run 按钮，或在 VSCode 的 MATLAB terminal 中运行命令。

如果 VSCode 没有自动找到 MATLAB，需要在 VSCode 设置中配置 MATLAB 安装路径。

## matplotlib 安装

当前系统环境中 `python3 -m pip` 不可用，所以不能直接 `pip install matplotlib`。可以在 VSCode 普通终端里执行：

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-matplotlib
python3 -c "import matplotlib; print(matplotlib.__version__)"
```

当前脚本已经能直接输出 SVG 图，所以安装 `matplotlib` 不是运行本工具的必要条件。
