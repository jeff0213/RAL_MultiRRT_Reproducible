# RAL-MultiRRT 可复现实验项目

本项目是论文 **《Risk-Aware Look-Ahead Planning for Independent Multi-Goal UAV Paths》** 的二维静态路径规划实验完整复现工程。当前版本为 **精确碰撞检测修正版（Exact-Validator Rerun）**，是在删除旧实验缓存、修复路径穿越障碍问题后重新运行得到的最终实验包。

项目包含：固定地图、随机地图、5 类算法实现、完整原始实验数据、统计分析、消融实验、参数敏感性实验、代表性路径、论文结果图、碰撞检测 QA、路径二次审计及一键复现实验脚本。

---

## 1. 项目目标

论文研究任务为：

> 从同一个无人机起点出发，分别规划到多个目标点的独立无碰撞路径，并比较 RAL-MultiRRT 与多种采样式路径规划基线在路径质量、风险、安全距离、转向特征与计算代价方面的表现。

本项目重点解决了审稿过程中提出的以下问题：

1. 原实验固定地图样本量不足，部分结果仅有单次运行；
2. MultiRRT、RRT*、RRT*-Smart 等基线缺少完整重跑；
3. 旧版 “APF-MultiRRT” 标签与实际执行算法存在不一致风险；
4. APF 与 APF + Look-Ahead 缺乏严格受控比较；
5. 原压力测试没有以独立地图作为统计实验单位；
6. 参数权重、look-ahead 预算、候选点数等缺少敏感性分析；
7. 路径转向指标受采样点密度影响；
8. 代码、地图、路径和原始实验数据不完整，难以独立复现；
9. 旧版线段碰撞检测采用有限距离采样，存在 corner-cutting / tunneling 漏检风险；
10. 修改地图后旧路径缓存可能继续被绘图程序读取，造成“旧路径 + 新地图”错配。

当前版本已针对以上问题全部重新设计、实现和验证。

---

## 2. 当前最终版本的重要修正

### 2.1 删除旧结果并完整重跑

在本次最终重跑前，已删除旧的：

- `data/raw/`
- `data/paths/`
- `data/maps/`
- `results/tables/`
- `results/figures/`

随后重新执行地图生成、正式 benchmark、随机地图实验、消融实验、敏感性分析、统计分析、路径审计和绘图流程。

因此，当前结果不再混用早期缓存数据。

### 2.2 精确连续几何碰撞检测

当前碰撞检测版本：

```text
exact_geometry_v2
```

旧版固定距离采样式线段检测已移除。

当前 validator 使用连续解析几何：

- **矩形障碍物**：检测线段与“矩形 + 10 m 安全圆盘”构成的 Minkowski 安全包络是否相交；
- **圆形障碍物**：使用线段到圆心的精确最短距离，与 `障碍半径 + safety_radius` 比较；
- **地图边界**：按 10 m safety radius 向内收缩；
- **擦边/相切**：按照碰撞处理，而不是视为可行路径。

同一个 validator 被统一用于：

- RRT / RRT* 树边扩展；
- APF 候选点生成；
- RAL look-ahead rollout；
- shortcut / refinement；
- 最终路径有效性检查；
- 绘图前路径二次审核。

### 2.3 精确几何 QA

文件：

```text
results/geometry_QA.json
```

当前验证结果：

- 矩形角点切穿回归测试：PASS
- 圆形障碍 tunneling 回归测试：PASS
- 10,000 条随机线段独立解析几何交叉验证：**10,000 / 10,000 PASS**
- mismatch：**0**

### 2.4 代表性路径二次审计

每个保存的路径 JSON 都包含：

```text
map_id
scenario_hash
algorithm
seed
validator_version
paths
```

绘图程序会检查：

- 地图 hash 是否与当前场景一致；
- algorithm label 是否一致；
- validator version 是否一致；
- 是否包含完整 3 条目标路径；
- 所有路径是否再次通过 exact geometry validator。

文件：

```text
results/path_audit.json
results/tables/saved_path_exact_audit.csv
```

当前结果：

```text
20 组代表性 path-set 被检查
20 / 20 PASS
0 FAIL
```

因此最新论文结果图不会再出现“旧路径画到新地图”或“路径直接穿越障碍”的问题。

---

## 3. 固定实验地图

固定地图大小统一为：

```text
1000 m × 800 m
```

每张地图包含：

- 1 个共同起点；
- 3 个独立目标点；
- 10 m safety radius；
- 完整公开的障碍物与目标坐标。

### S1：标准混合障碍场景

```text
12 个矩形障碍 + 4 个圆形障碍
```

特点：混合障碍、错位屏障、多条可行绕行路线。

### S2：结构化走廊场景

```text
16 个矩形障碍
```

特点：多组错位屏障、走廊和缺口，适合测试不同采样算法的通道选择能力。

### S3：APF 挑战场景

```text
10 个矩形障碍 + 8 个圆形障碍
```

特点：混合障碍、局部诱导区域、窄连接和多路径结构，用于检验 APF 引导及 finite-horizon look-ahead 的实际作用。

### S4：高密度圆形障碍场景

```text
21 个圆形障碍
```

特点：密集圆形障碍簇和多条同伦路径选择。

地图定义位于：

```text
scenarios/S1.json
scenarios/S2.json
scenarios/S3.json
scenarios/S4.json
```

论文风格地图总览：

```text
results/figures/fixed_maps_overview.png
```

图中的橙色虚线表示 **10 m 精确 safety envelope**，不是装饰线。

---

## 4. 独立随机地图实验

随机地图分为：

```text
D1
D2
D3
Narrow
```

其中：

- D1：10 个障碍物；
- D2：18 个障碍物；
- D3：26 个障碍物；
- Narrow：错位墙体 + 显式窄通道。

D1 / D2 / D3 使用独立生成的混合障碍地图，不把同一张地图重复运行简单当作独立样本。

所有随机地图保存在：

```text
data/maps/
```

这些地图用于独立 map-level replication，避免 pseudo-replication。

---

## 5. 实现的算法

当前正式 benchmark 包含 5 种方法：

```text
MultiRRT
RRTStar
RRTStarSmart
APFMultiRRT
RALMultiRRT
```

对应含义：

- **MultiRRT**：针对三个目标分别执行独立 RRT 规划；
- **RRTStar**：加入邻域最优父节点和 rewiring；
- **RRTStarSmart**：在 RRT* 基础上增加安全 shortcut 优化；
- **APFMultiRRT**：真正调用 APF 引导的 MultiRRT 候选扩展；
- **RALMultiRRT**：与 APF-MultiRRT 共享相同 APF-guided tree expansion，仅额外启用有限时域 look-ahead scoring。

特别说明：

> 当前 APF-MultiRRT 和 RAL-MultiRRT 中均不存在隐藏 A* 或 grid-recovery 分支。

因此 APF-MultiRRT 与 RAL-MultiRRT 的受控差异主要是：

```text
是否启用 finite-horizon rollout scoring
```

这直接对应审稿人提出的“前瞻机制到底有没有独立贡献”的问题。

---

## 6. 当前最终实验规模

本次 clean rerun 共包含：

```text
796 个 path-set attempts
```

其中：

| 实验 | 数量 |
|---|---:|
| 固定地图正式 benchmark | 400 |
| 独立随机地图 benchmark | 300 |
| Look-Ahead `(M,H)` 消融 | 30 |
| Candidate Count / APF Radius 敏感性 | 27 |
| Turning / Risk Weight 敏感性 | 27 |
| APF Repulsion Gain `krep` 敏感性 | 12 |
| **合计** | **796** |

每个 path-set attempt 都独立规划：

```text
Start → Goal 1
Start → Goal 2
Start → Goal 3
```

因此每次实验对应 3 条独立路径。

核心 benchmark 共：

```text
400 + 300 = 700 个正式 path-set
```

辅助参数 sweep 使用明确的有限迭代预算，目的是证明参数敏感性，不用于做全局算法排名。

---

## 7. 当前主要实验结论

### 7.1 APF-MultiRRT vs RAL-MultiRRT

两种方法共享：

- APF-guided tree expansion；
- candidate generator；
- validator；
- refinement；
- safety radius；
- map / seed 配对条件。

RAL 额外启用 look-ahead rollout。

当前 jointly-valid 配对结果（`RAL - APF`）：

### 固定地图

平均路径长度差：

```text
+1.66 m
```

Bootstrap 95% CI：

```text
[-8.48 m, 12.32 m]
```

说明：固定地图整体上不能证明 RAL 的平均路径长度稳定优于 APF。

### 独立随机地图

平均路径长度差：

```text
-10.68 m
```

Bootstrap 95% CI：

```text
[-18.52 m, -2.52 m]
```

说明：在独立随机地图中，当前实验观察到 RAL 平均路径长度有所下降。

与此同时，RAL 在两类实验中的运行时间都明显增加。

因此当前合理结论是：

> **有限时域前瞻体现出环境相关的路径质量—计算成本权衡，而不是在所有地图、所有指标上全面优于 APF。**

不能把实验结论扩大成“RAL 在所有场景中始终最优”。

详细结果：

```text
results/tables/apf_vs_ral_effects.csv
results/tables/pairwise_permutation_tests.csv
```

---

## 8. 评价指标

当前实验主要记录：

- success / valid rate；
- mean path length；
- minimum clearance；
- turning-angle proxy；
- curvature proxy；
- runtime；
- samples；
- expanded nodes；
- collision checks；
- validator calls；
- APF calls；
- look-ahead trials。

转角指标不是直接使用算法原始 waypoint，而是先进行统一的：

```text
15 m arc-length resampling
```

再进行比较，从而降低“不同算法路径点密度不同”造成的指标偏差。

---

## 9. 消融与参数敏感性实验

### Look-Ahead Budget

对应文件：

```text
data/raw/lookahead_ablation.csv
results/tables/lookahead_ablation_summary.csv
```

用于分析：

- rollout 数量 M；
- horizon H；
- 路径质量；
- 运行时间。

实验用于回答：增加前瞻预算后，计算成本如何增加，以及路径质量是否随预算单调改善。

### Candidate Count / APF Radius

```text
data/raw/sensitivity_C_rho.csv
results/tables/sensitivity_C_rho_summary.csv
```

主要分析：

- Candidate Count `C`
- APF Influence Radius `rho`

### Turning / Risk Weight

```text
data/raw/sensitivity_weights.csv
results/tables/sensitivity_weights_summary.csv
```

主要分析：

- turning weight；
- risk weight。

### APF Repulsion Gain

```text
data/raw/sensitivity_krep.csv
results/tables/sensitivity_krep_summary.csv
```

主要分析：

```text
krep
```

这些参数实验用于说明选定参数经过敏感性检查，而不是宣称某一组参数是所有环境下的“全局最优值”。

---

## 10. 统计分析

当前统计输出包括：

### Bootstrap Confidence Interval

```text
results/tables/fixed_bootstrap_ci.csv
results/tables/apf_vs_ral_effects.csv
```

### Paired Random-Sign Permutation Test

```text
results/tables/pairwise_permutation_tests.csv
```

并使用 Holm correction 对预定义比较族进行多重比较校正。

对于随机地图实验，统计单位优先采用独立地图，而不是把同一地图中的重复运行直接当作完全独立样本。

---

## 11. 结果图

当前论文级图形均位于：

```text
results/figures/
```

主要包括：

```text
fixed_maps_overview.png
S1_algorithm_comparison.png
S2_algorithm_comparison.png
S3_algorithm_comparison.png
S4_algorithm_comparison.png
S1_RAL_paths.png
S2_RAL_paths.png
S3_RAL_paths.png
S4_RAL_paths.png
fixed_path_length_boxplot.png
cost_quality_scatter.png
```

特点：

- 地图版式统一；
- 障碍物填充与边框风格统一；
- 起点 / 目标点统一标记；
- 显示 10 m safety envelope；
- 路径图生成前必须通过 exact validator；
- 不允许无效路径进入论文图。

---

## 12. 项目目录结构

```text
RAL_MultiRRT_Reproducible/
│
├─ src/
│  ├─ core.py
│  ├─ geometry.py
│  ├─ maps.py
│  ├─ metrics.py
│  ├─ planners.py
│  └─ refinement.py
│
├─ scenarios/
│  ├─ S1.json
│  ├─ S2.json
│  ├─ S3.json
│  └─ S4.json
│
├─ experiments/
│  ├─ run_fixed.py
│  ├─ run_random.py
│  ├─ run_ablation.py
│  ├─ run_sensitivity.py
│  ├─ run_weight_sensitivity.py
│  ├─ run_krep_sensitivity.py
│  └─ run_all.py
│
├─ analysis/
│  ├─ analyze.py
│  ├─ combine_results.py
│  ├─ statistics_extra.py
│  └─ plot_paths.py
│
├─ tests/
│  ├─ test_core.py
│  └─ test_exact_geometry.py
│
├─ data/
│  ├─ raw/
│  ├─ maps/
│  ├─ paths/
│  └─ processed/
│
├─ results/
│  ├─ figures/
│  ├─ tables/
│  ├─ geometry_QA.json
│  ├─ path_audit.json
│  └─ EXPERIMENT_REPORT.md
│
├─ reproduce_all.py
├─ requirements.txt
├─ environment.yml
├─ VALIDATION_AND_RERUN_NOTE.md
├─ RESULTS_INDEX.md
├─ REVIEWER_REQUIREMENTS_MAPPING.md
├─ MANUSCRIPT_UPDATE_GUIDE.md
├─ README.md
└─ README_中文.md
```

---

## 13. 环境安装

推荐使用独立 Python 环境。

安装依赖：

```bash
python -m pip install -r requirements.txt
```

也可以参考：

```text
environment.yml
```

---

## 14. 一键复现全部实验

在项目根目录运行：

```bash
python reproduce_all.py
```

该脚本会依次：

1. 清理旧生成结果；
2. 运行 exact geometry QA；
3. 重新生成固定地图与随机地图实验；
4. 运行 5 种算法 benchmark；
5. 运行 look-ahead 消融；
6. 运行参数敏感性实验；
7. 生成统计表；
8. 对代表性路径再次执行 exact validator audit；
9. 只有全部路径审核通过后才生成论文结果图。

因此，如果未来代码修改造成路径穿过障碍，流程应直接失败，而不是继续输出错误论文图。

---

## 15. 主要数据文件说明

### 固定地图原始结果

```text
data/raw/fixed_runs.csv
```

### 独立随机地图原始结果

```text
data/raw/random_runs.csv
```

### 全部正式 benchmark

```text
data/raw/all_runs.csv
```

### Look-Ahead 消融

```text
data/raw/lookahead_ablation.csv
```

### 参数敏感性

```text
data/raw/sensitivity_C_rho.csv
data/raw/sensitivity_weights.csv
data/raw/sensitivity_krep.csv
```

### 保存的代表性原始路径

```text
data/paths/
```

### 统计汇总表

```text
results/tables/
```

---

## 16. 结果解释边界

本项目当前证明的范围是：

> **静态二维几何环境中的多目标独立无人机路径规划。**

当前实验**不能**直接证明：

- 真实三维飞行动力学可行性；
- 动态障碍物规避能力；
- 飞控执行性能；
- 加速度、速度和最小转弯半径约束；
- 遥感图像质量；
- 真实飞行任务中的覆盖质量；
- 环境无关的普适可靠性。

因此论文结论应限定在当前证据能够支持的二维静态几何规划与验证范围内。

---

## 17. 当前完成状态

当前项目已完成：

- [x] 复杂固定地图 S1–S4 重建
- [x] 独立随机地图数据生成
- [x] MultiRRT 实现
- [x] RRT* 实现
- [x] RRT*-Smart 实现
- [x] 真正 APF-MultiRRT 实现
- [x] RAL-MultiRRT finite-horizon look-ahead 实现
- [x] APF vs RAL 严格受控比较
- [x] exact geometry validator
- [x] 10,000 条独立碰撞 QA
- [x] 旧结果缓存清理机制
- [x] scenario hash 防错配机制
- [x] 20 / 20 代表性路径二次审计
- [x] 固定地图正式实验
- [x] 独立随机地图正式实验
- [x] Look-Ahead 消融实验
- [x] C / rho 参数敏感性
- [x] turning / risk 权重敏感性
- [x] krep 敏感性
- [x] Bootstrap 置信区间
- [x] permutation test + Holm correction
- [x] 论文风格地图与路径图
- [x] 完整原始 CSV 数据
- [x] 完整地图与路径 JSON
- [x] 一键复现实验脚本
- [x] 审稿意见对应说明
- [x] 论文修改说明

