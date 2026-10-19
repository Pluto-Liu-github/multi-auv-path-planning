# 实验与复现说明

## 默认三维参数

| 参数 | 无洋流 `mainin.m` | 有洋流 `mainin_3d_current.m` |
| --- | --- | --- |
| 栅格范围 | 25 × 25 × 25 | 25 × 25 × 25 |
| AUV / 目标数量 | 5 / 6 | 5 / 6 |
| `V_AUV` | 2.0 | 2.0 |
| `gamma_penalty` | `1e-4` | `1e-4` |
| 最大主循环次数 `Step` | 30000 | 30000 |
| 活性扩散次数 | 显式传入 30 | 函数默认值 100 |
| 目标到达距离阈值 | 3.5 | 0.8 |
| 水平流速尺度 `V_current` | — | 0.25 |
| 垂向扰动 `V_current_vertical` | — | 0.02 |
| 顺流评分增益 `current_guidance_gain` | — | 0.000 |

`Robot_1`、`Target_12` 每列分别表示一个起点或目标的 `[X; Y; Z]`。障碍物由坐标循环构建。部分制图脚本单独定义了地图，修改场景时需要同步核对。

`weight_wij.m`、`weight_quanju.m` 接受可选扩散次数参数，不传入时默认 100。活性传播包含多层循环，耗时取决于机器与参数。

## 主要模块

- `matlab/core/weight_wij.m`、`weight_quanju.m`：局部、全局活性场。
- `matlab/core/winner_robot.m`、`MUL_AUV.m`：竞争分配及任务关系处理。
- `matlab/core/Pn.m`、`A1_updateWeight.m`：候选节点与位置更新。
- `matlab/current/make_current_field_3d.m`：随深度变化的剪切流、中央射流与弱垂向扰动。
- `matlab/current/apply_current_drift_3d.m`：位置漂移，关闭顺流节点评分不会关闭此步骤。
- `matlab/baselines/run_baseline_comparison.m`：按任务配对逐段执行 A*、APF，同一 AUV 从上一段结束位置继续。

## 辅助绘图与数据

以下脚本在项目根目录运行，输出写入 `paper-output/figures/`：

| 脚本 | 用途 / 输入 |
| --- | --- |
| `generate_fig5_baseline_only.m` | 更新 A*、APF 路径图和任务配对 CSV，不刷新完整 GBSOM 指标 |
| `generate_obstacle_three_views_matlab.m` | 导出障碍物顶视、正视、侧视图 |
| `generate_2d_environment_matlab.m` | 导出 `fig2d-many-to-many-restored-screenshot-version.png` |
| `generate_2d_energy_distance_analysis.m` | 读取 `fig3-2d-current-metrics.mat` 和 `fig3-2d-no-current-metrics.mat`，导出有／无洋流能耗与航程比较 |
| `generate_2d_current_original_vs_planned_energy.m` | 读取 `fig3-2d-current-planned-run.mat`，按任务记录分别以距离、能耗代价运行二维 A* |

最后一项并非直接导出 GBSOM 原始轨迹，不能仅凭名称将其解释为 GBSOM 改进前后对比。上述 MAT 文件是现存数据快照；修改参数后应重新生成并记录输入。

`matlab/tmp_2d_*` 保留了无洋流、有洋流和能耗分析工作副本。它们与 `original_2d` 含有同名函数，需分目录运行；目录名 `original_2d` 不代表文件从未修改。

## 比较口径

1. 无洋流入口根据 GBSOM 到达记录生成基线任务配对，未记录到达的目标按最近初始 AUV 补齐。有洋流入口没有传入配对，基线按目标到各 AUV 初始位置的距离分配。
2. 有洋流入口的基线函数不接收流场，其等效能耗仍按成功路径总长度乘以 `V_AUV^2` 计算；GBSOM 有洋流能耗包含流速方向、航行时间与相对消耗的简化估计。
3. GBSOM 无／有洋流入口的到达距离阈值分别为 3.5、0.8。APF 距目标不超过 1 时追加目标节点并成功返回；A* 要求到达目标节点。
4. `SearchSteps` 分别是 GBSOM 主循环次数、A* 搜索展开次数、APF 迭代次数，不是统一时间指标。
5. 基线总路径长度、平均路径长度只汇总成功任务；GBSOM 平均路径长度是累计航程除以目标总数。发生任务失败时应同时报告成功率。
6. 能耗为模型中的等效值，没有真实推进系统标定，不能直接解释为焦耳、电池容量或实测续航。

## 建议复现流程

1. 使用独立 MATLAB 会话，检查函数解析路径，记录 MATLAB 版本。
2. 固定地图、起点、目标、任务配对及到达判据。
3. 若对比有／无洋流，统一扩散步数等非洋流参数。
4. 分别保存参数、工作区数据、输出图表和运行时间；不要混用旧 MAT 数据与新参数。
5. 检查任务完成情况、路径与障碍物关系，保留失败任务的统计。

仓库中的图表和 CSV 来自历史文件，未在本次整理中重新运行验证，不作为自动化测试通过或算法性能保证。
