# Tunnel bag 与本地 TSDF 端到端验证

验证日期：2026-08-19。数据为现场 `Tunnel.bag`（仓库不记录机器本地绝对路径），运行环境为 ROS Noetic、PCL 1.10、Firefox 136 headless。本文的数值是当前工作树和当前 x86 主机的基线，不是 ARM64 目标机性能保证。

## 结论

当前数据链已经能够从双雷达融合点云生成 Marker，经新版 Bridge 的 `9003/mesh` 发送有效二进制包，并在 Web 页面显示本地 TSDF Mesh。但它还没有通过“实时、封闭隧道表面”的验收：

- 安全默认会跳过不可验证的 GICP，以原子单帧重建持续更新 TSDF；20 条状态全部为 `SINGLE_FRAME_FALLBACK`，且没有 DELETE/reset 闪断。
- 显式打开 `allow_unverified_bootstrap=true` 后可以形成 4～6 帧的滚动体，并观察到过期帧淘汰；这个模式只适合视觉实验，因为 Tunnel bag 中的纵向运动对纯雷达 ICP 不可辨识。
- 安全单帧回退在每帧提取并发布 Mesh 时，回调耗时中位数约 `41.2 ms`、p95 约 `45.6 ms`，实测 ROS Marker 和 9003 都约 10 Hz；不安全多帧实验仍为中位 `305 ms`、p95 `422 ms`，达不到 10 Hz。
- 网格会触及 `200000` 三角形硬上限；实测快照存在大量边界和碎片，5 个纵向截面没有一个闭合。因此“能显示”不等于“已封闭”。

## Bag 输入基线

Bag 大小约 9.53 GB，时长 194.997 s，不含高频里程计或可用 TF 轨迹真值。

| Topic | 采样帧 | 频率 | 每帧点数中位数 | ring | 0.5～80 m 有效率 |
|---|---:|---:|---:|---:|---:|
| `/velodyne_points` | 120 | 9.991 Hz | 65697 | 32 | 99.975% |
| `/right/velodyne_points` | 120 | 9.915 Hz | 13606 | 16 | 99.903% |
| `/left/lslidar_point_cloud` | 120 | 9.998 Hz | 22827 | 无 ring 字段 | 69.694% |

中心到右雷达最近 header stamp 偏差中位数为 23.38 ms、p95 为 43.94 ms。当前融合 demo 使用中心和右雷达，并应用仓库现场给定的真实右雷达外参。现场录得的融合 `/points_raw` 为约 9.88 Hz、每帧点数中位数 78053，包含：

```text
x y z intensity ring lidar_id time azimuth range
```

这个 bag 的中心雷达为 32 ring、右雷达为 16 ring，因此可用于检验隧道退化和端到端吞吐，但不能替代“双 TM16 完全同型号”的最终现场验证。

## 纯雷达配准可行性

### Identity 初值

对中心 `/velodyne_points` 的 100 个相邻帧对执行 point-to-point ICP，参数为 0.25 m voxel、1.0 m 最大对应距离、40 次迭代：

| 指标 | 结果 |
|---|---:|
| converged | 100/100 |
| 旧质量门控判定 plausible | 100/100 |
| fitness 中位数 / p95 | 0.01632 / 0.02882 m² |
| overlap 中位数 / p10 | 99.60% / 98.46% |
| 输出位移中位数 / p95 | 0.0193 / 0.0458 m |
| ICP 耗时中位数 / p95 | 53.0 / 180.7 ms |

这些漂亮的数值是错误近零解，不是里程计精度。以真实外参融合中心和右雷达后再测 50 对，仍然 100% 收敛，位移中位数反而降至 0.0083 m；双雷达没有解除纵向退化。

### 多初值验证

对同 100 对点云使用 0.40 m voxel、2.5 m 最大对应距离和 30 次迭代。每对分别沿固定 X 轴和点云 PCA 主轴施加 `-2.0～2.0 m`、步长 0.5 m 的初值，共 18 次 ICP。PCA 主轴约为 `(0.2, -0.98, -0.01)`，即真实隧道长轴主要接近 Y，而不是 X。

| 指标 | 结果 |
|---|---:|
| 全局最低 fitness 解的最终位移中位数 / p95 | 0.0273 / 0.0687 m |
| identity fitness 位于 best + 0.002 m² 内 | 100/100 |
| X 轴 9 个初值全部位于 best + 0.002 m² 内 | 中位 9/9 |
| 18 次串行 ICP 每对耗时中位数 / p95 | 1.99 / 2.89 s |

沿 PCA 做 `-0.5/0/+0.5 m` 三探针时，最终变换 spread 的 p10/中位/p90/p95 为 `0.284/0.303/0.344/0.591 m`，fitness 最大最小差为 `0.00945/0.01698/0.02455/0.02727 m²`。可将“spread > 0.20 m 且 fitness 差 < 0.03 m²”作为当前 bag 的启动期退化报警参考，但它必须参数化，不能宣称是通用阈值。

### INSPVAX 的正确解释

Bag 中 `/novatel_data/inspvax` 只有 1 Hz，全部 `ins_status=0`，其消息定义对应 `INS_STATUS_INACTIVE`。它只能证明载体明显在运动，不能当轨迹真值，更不能据此标定 ICP 误差。

此前“约 1.14 m/帧”和本次“约 0.436 m/帧”并不矛盾：

- `1.14 m/帧` 是整段 bag 的 INSPVAX 速度中位数 `11.418 m/s` 直接乘名义 `0.1 s`。
- `0.436 m/帧` 是 ICP 所取 bag 起始 100 对的逐帧时间戳，与该时段约 `4～5 m/s` 的最近 1 Hz INSPVAX 速度对齐后得到的中位数。
- 两者都是速度模长乘时间，没有投影到可靠的隧道轴或车辆轨迹，而且 INS 为 INACTIVE；报告只用它们作为“近零 ICP 与明显运动不一致”的旁证。

退化结论主要由多初值同成本、正确量级运动未形成优势极小值，以及融合云仍回到近零解共同支撑。

## TSDF 端到端结果

测试链路：

```text
Tunnel.bag center + right
  -> multi_lidar_fusion + 真实右雷达外参
  -> /points_raw
  -> local_tsdf_mesh_node
  -> /local_tsdf_mesh/mesh + /local_tsdf_mesh/status
  -> mine_web_bridge_node
  -> ws://127.0.0.1:9003/mesh
  -> http://127.0.0.1:5173
```

### 安全默认

运行时确认：

```text
registration/allow_unverified_bootstrap = false
registration/allow_near_identity_bootstrap = false
registration/safe_single_frame_fallback = true
```

连续 30 条 status：

| 指标 | 结果 |
|---|---:|
| `SINGLE_FRAME_FALLBACK:unverified_motion` | 30/30 |
| active frames | 恒为 1 |
| active voxels 中位数 | 60404 |
| status mesh triangles中位数 / p95 | 48633 / 52212 |
| registration attempted / accepted | 0/30 / 0/30 |
| registration 中位数 / p95 | 0 / 0 ms |
| integration 中位数 / p95 | 10.64 / 11.39 ms |
| extraction 中位数 / p95 | 25.09 / 28.75 ms（每帧） |
| total 中位数 / p95 | 41.24 / 45.63 ms |
| status/Marker 间隔中位数 | 100.18 ms |
| rejected frames / reset | 0 / 0 |

安全回退按预期工作：它不再执行注定退化的 GICP，也不再累计“三次失败 → DELETE → reset”。每个输入帧在临时 volume 中重建，成功后原子替换当前单帧体。

6 s 录制中收到 56 个 status 和 56 个 Marker，二者均为 `9.852 Hz`；Marker 全部为 ADD，DELETE 为 0。三角形中位数 44727、p95 51552、最大 53250、变异系数 6.67%，相邻帧数量比 p10/p90 为 `0.974/1.022`，没有异常归零。9003 另取 30 包全部为 replace，revision 严格连续 `948～977`，到达率 `10.044 Hz`；包大小中位数 1.164 MiB、p95 1.720 MiB，payload 吞吐约 12.42 MiB/s。

5 s `/proc` 采样中，TSDF 节点使用约 49.6% 单核 CPU、RSS 约 87.5～95.1 MiB；Bridge 使用约 8.6% 单核 CPU、RSS 约 47.2～47.6 MiB。这是当前 x86 主机、同时播放 bag 和渲染 Web 时的基线。每帧发布没有旧实现的周期性 clear/reset 闪烁；单帧几何本身的细小变化仍需现场肉眼判断。

Firefox 页面实测 cloud/status/Mesh 三个连接均为 true，`tsdfMeshPacketErrors=0`，TSDF Mesh revision 持续更新。这个模式性能达标且显示稳定，但仍是单帧表面，不应表述为多帧封闭重建。

### 单帧覆盖调优

为改善远处激光线在 10 cm 体素下难以形成完整邻域的问题，当前配置调整为：

```yaml
tsdf:
  voxel_size: 0.15
  truncation_distance: 0.45
  max_points_per_frame: 18000
```

隔离的连续 10 帧对照中，表面积比原配置增加约 183%，纵向包围盒增加约 12%，boundary edge ratio 从 21.60% 降至 12.59%，总耗时 p95 为 47.34 ms。在线 30 帧复核全部积分成功且没有 DELETE、reject 或 triangle limit；积分射线中位数从 11128 增至 15210，Mesh/有效点云在 x/y/z 三轴的范围覆盖约从 `90.0%/75.5%/55.2%` 提升到 `94.6%/83.4%/63.5%`。

真实浏览器在固定视口、关闭实时点云后进行 TSDF on/off 像素差分，有效像素从 153 增至 366，约为原来的 2.39 倍，屏幕包围盒从 `28×17` 增至 `42×22`。该调整扩大的是单帧可观察表面，不改变“无可信位姿时不做多帧累计”的安全约束。

单独增大 `max_range` 在当前数据上几乎没有收益；将射线预算设为 24000 还会因为固定 stride 与有序点排列混叠而退化，因此没有采用。后续若继续提高射线预算，应先改为按雷达、ring 和方位角分层均匀采样。

### 显式不安全视觉实验

运行时明确设置：

```text
registration/allow_unverified_bootstrap = true
registration/allow_near_identity_bootstrap = false
```

干净的连续 20 条 status（不跨 rosbag loop 回卷）：

| 指标 | 中位数 | p95 / 最大值 |
|---|---:|---:|
| TRACKING accepted | 20/20 | 20/20 |
| active frames | 5 | 最大 6 |
| active voxels | 145426 | p95 155445 |
| evicted frames / callback | 1 | 最大 2；20 条合计 20 |
| fitness | 0.01863 m² | p95 0.02235 |
| inlier ratio | 95.28% | p95 97.89% |
| relative translation | 0.164 m | p95 0.394 m |
| registration | 179.9 ms | p95 354.3 ms |
| integration | 51.9 ms | p95 63.7 ms |
| extraction | 35.2 ms | p95 104.7 ms |
| total | 305.1 ms | p95 421.6 ms |

Marker 采到 17 个完整快照，三角形中位数 182106，最大值 200000。status 中 10% 的样本报告 `mesh_triangle_limit_reached=true`。滚动淘汰和上限均生效，但当前 CPU 负载使实际回调约为 3 Hz，而不是输入的约 10 Hz。

这个开关不会让错误 ICP 变正确。它仅证明 TSDF、淘汰、网格、Bridge 和浏览器链路能够工作，不能用于定位或安全控制。

## WebSocket 和浏览器证据

标准库 RFC 6455 探针从 `9003/mesh` 连续收到两个有效 `MMSH v1` replace 包：

| revision | vertices | triangles | bytes | frame |
|---:|---:|---:|---:|---|
| 408 | 406107 | 135369 | 6497784 | `velodyne` |
| 409 | 421824 | 140608 | 6749256 | `velodyne` |

探针核对了 magic、version、operation、revision、packet bytes、frame id、顶点数为 3 的倍数，以及 positions/colors 边界。`9002/status` 同时报告 Mesh revision、三角形数、包大小和 source topic。

Firefox 136 headless 实际加载 `http://127.0.0.1:5173/` 后，通过 WebDriver BiDi 读取页面运行态：

```text
cloudConnected       true
statusConnected      true
tsdfMeshConnected    true
tsdfMeshPacketErrors 0
tsdfMeshRevision     525
tsdfMeshTriangles    38744
render FPS           约 60
```

页面正文中的三个 WebSocket 徽标也均为“在线”。生产前端构建成功；主 bundle 约 631 kB，Vite 只给出 chunk 大小警告。当前前端测试为 12 pass、0 fail、1 个显式 TODO；TODO 是扫描条带跨 `-π/π` 方位角 seam 的后续修复，不应误记成已覆盖能力。

另外对断线安全语义做了真实浏览器验证：

- 停止 `local_tsdf_mesh_node` 后重启新版 Bridge，9003 新连接的首包为 `clear`、`revision=1`、64 bytes。
- 再启动安全默认的 Mesh 节点，新连接收到 `replace`，revision 已递增且包含有效三角形。
- 浏览器已有 36705 个 TSDF 三角形时杀掉 Bridge，2 s 后页面状态变为 `tsdfMeshConnected=false`，本地层立即变为 `triangle_count=0`、`last_operation=clear`，没有残留旧 geometry。
- 重启 Bridge 后页面自动恢复三个 WebSocket，Mesh 重新收到递增 revision 的 replace 包，`tsdfMeshPacketErrors=0`。

这验证了“无生产者也有权威空状态”和“断线不保留幽灵 Mesh”的端到端行为，而不只是对应单元测试。

最新 publisher/watchdog 也做了独立测试，期间只终止 TSDF producer，Bridge 和 Firefox 始终保持连接：

- kill 前 9003 为 replace revision 127、47061 三角形。
- `rosnode kill /local_tsdf_mesh_safe` 发起后 2.251 s（命令返回后 2.005 s），9003 收到 clear revision 130；中间只发送了两个已排队/在途的 replace revision 128、129。
- Bridge 状态同时变为 publisher count 0、triangle count 0、packet bytes 64。
- Firefox 的 Mesh WebSocket 仍为 connected，几何变为 0 三角形、`last_operation=clear`、packet error 仍为 0。
- 重启安全 producer 后，publisher count 恢复为 1，9003 收到 replace revision 200、45581 三角形；Firefox 随后恢复 replace revision 295、51954 三角形，三个 WebSocket 均保持在线。

因此 publisher 消失不会立即清空仍可能有效的最后快照，而是等待配置的 2 s stale timeout；实测延迟与该语义一致，且恢复不要求重启 Bridge 或刷新页面。

## 闭合与边界指标

对一个 Marker 快照进行 2 mm 顶点焊接并沿 PCA 长轴取 5 个内部截面：

| 指标 | 结果 |
|---|---:|
| 输入三角形 | 154552 |
| 非退化三角形 | 153211 |
| 退化三角形 | 1341 |
| boundary edges | 38143 |
| boundary edge ratio | 15.33% |
| non-manifold edges | 4 |
| connected components | 2511 |
| 最大分量面积占比 | 76.58% |
| watertight | false |
| 闭合纵向截面 | 0/5 |

因此当前快照是开放且碎片化的表面，不满足封闭隧道横截面验收。三角形硬截断、短滚动窗、运动配准误差和真实无回波区都可能贡献边界；在定位退化和性能问题解决前，不应靠单独放宽 TSDF/Marching 参数掩盖这些现象。

## 建议验收矩阵

| 场景 | 输入 | 必须检查 | 建议通过条件 |
|---|---|---|---|
| 静止 | 同一帧重复 100 次或真实静止 bag | 位姿漂移、active frame cap、表面抖动 | 平移漂移 < 2 cm；角度漂移 < 0.2°；无 reset |
| 已知平移 | 合成封闭管廊 + 已知 6DoF | 配准误差、TSDF 重影 | 平移误差 < 5 cm；旋转误差 < 0.5° |
| 退化隧道 | 当前 Tunnel bag | 多初值成本、先验一致性 | 无可信先验时拒绝积分；不得仅以 fitness 放行 |
| 配准失败 | 空帧、少点、跳变 > gate | fallback/reset、体积污染 | rejected 计数递增；失败帧不改变 active volume |
| 滚动淘汰 | 超过 1.5 s 和 15 帧 | active frames/voxels、旧贡献 | frame 数不超上限；出现 eviction；旧区域贡献可减除 |
| 动态物体 | 静态隧道 + 移动物体 | 拖影寿命、组件数量 | 拖影不超过窗口；静态主组件不被动态面撕裂 |
| 闭合 | 合成方管/圆管 | boundary、nonmanifold、截面环 | 内部截面闭合率 100%；nonmanifold=0 |
| 100 帧性能 | 目标 ARM64 实机 | decode/register/integrate/extract/total | total p95 < 输入周期；不触发点/三角硬截断 |

当前安全单帧回退通过了 x86 的 100 ms 回调预算；不安全多帧模式没有通过性能项，实测 Mesh 也没有通过闭合项。不能把浏览器帧率当作后端多帧 10 Hz 达标。

## 复现命令

Bag 字段、频率和点数：

```bash
export TUNNEL_BAG=/path/to/Tunnel.bag

python3 tools/tunnel_bag_baseline.py \
  "$TUNNEL_BAG" \
  --frames 120 --output /tmp/tunnel_bag_baseline.json
```

编译及运行相邻帧 ICP 工具：

```bash
g++ -std=c++14 -O2 -Wall -Wextra -Wpedantic \
  tools/tunnel_icp_baseline.cpp -o /tmp/tunnel_icp_baseline \
  $(pkg-config --cflags --libs roscpp rosbag sensor_msgs pcl_conversions \
    pcl_registration-1.10 pcl_filters-1.10 pcl_kdtree-1.10 pcl_io-1.10)

/tmp/tunnel_icp_baseline "$TUNNEL_BAG" \
  --topic /velodyne_points --pairs 100 --csv /tmp/center_icp.csv
```

记录并检查 100 条 TSDF 状态：

```bash
rosbag record -O /tmp/local_tsdf_validation.bag \
  /local_tsdf_mesh/status /local_tsdf_mesh/mesh

python3 tools/tsdf_bag_metrics.py /tmp/local_tsdf_validation.bag \
  --messages 100 --max-active-frames 15 --max-triangles 200000 \
  --budget-ms 100 --output /tmp/local_tsdf_metrics.json
```

检查 Marker 边界和截面：

```bash
python3 tools/mesh_topology_metrics.py \
  --bag /tmp/local_tsdf_validation.bag \
  --topic /local_tsdf_mesh/mesh --marker-index -1 \
  --axis pca --stations 9 --output /tmp/local_tsdf_topology.json
```

验证 Bridge 的真实 WebSocket 包：

```bash
python3 tools/websocket_mesh_probe.py \
  ws://127.0.0.1:9003/mesh --kind mesh --messages 2

python3 tools/websocket_mesh_probe.py \
  ws://127.0.0.1:9002/status --kind status --messages 1
```

所有脚本只读 bag/网络，JSON 输出位置由调用者显式指定；仓库不保存现场 bag 或机器本地结果。
