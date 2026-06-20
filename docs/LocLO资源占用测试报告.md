# LocLO 资源占用测试报告

测试日期：2026-06-20  
测试对象：`loc_lo_node`  
测试数据：`/home/guoli/data/yangpu/loc/merged_bag.bag`

## 1. 测试目的

本次测试用于评估 `Loc_LO.cpp` 在纯 LO 模式下的资源占用情况。测试时关闭 RViz、调试点云、全局地图显示和录包，尽量只观察算法节点本身的 CPU、内存、IO 和输出频率。

## 2. 测试配置

启动配置等价于：

```bash
roslaunch genz_icp loc_lo.launch \
  rviz:=false \
  visualize:=false \
  publish_global_map:=false \
  record:=false
```

实际运行时为了避免当前终端环境中 `roslaunch` 日志检查/输出阶段卡住，使用了：

```bash
roslaunch --skip-log-check genz_icp loc_lo.launch \
  rviz:=false \
  visualize:=false \
  publish_global_map:=false \
  record:=false
```

播放命令：

```bash
rosbag play /home/guoli/data/yangpu/loc/merged_bag.bag --clock
```

bag 信息：

| 项目 | 数值 |
| --- | --- |
| 时长 | 368.9 s |
| 文件大小 | 8.4 GB |
| 解压后数据量 | 26.5 GB |
| `/lidar_preprocessor/meta_cloud` | 3689 帧，约 10 Hz |
| `/localization/ins` | 33698 帧，约 91 Hz |

节点参数要点：

| 参数 | 值 |
| --- | --- |
| `odom_frame` | `map` |
| `use_ins_init` | `true` |
| `visualize` | `false` |
| `publish_global_map` | `false` |
| `record` | `false` |
| `config_file` | `outdoor.yaml` |
| `voxel_size` | `0.6` |
| `desired_num_voxelized_points` | `3000` |
| `max_num_iterations` | `100` |

## 3. 采集方法

采集命令：

```bash
pidstat -urd -p <loc_lo_node_pid> 1
rostopic hz /genz/odometry
/usr/bin/time -v rosbag play /home/guoli/data/yangpu/loc/merged_bag.bag --clock
```

原始采样文件：

```text
/tmp/loc_lo_resource_test/pidstat_loc_lo.txt
/tmp/loc_lo_resource_test/rostopic_hz_odometry.txt
/tmp/loc_lo_resource_test/rosbag_play_time.txt
/tmp/loc_lo_resource_test/rosbag_play.log
```

## 4. 测试结果

### 4.1 输出频率

`/genz/odometry` 输出频率：

| 指标 | 数值 |
| --- | --- |
| 平均频率 | 9.935 Hz |
| 最小间隔 | 0.011 s |
| 最大间隔 | 0.319 s |
| 标准差 | 0.0485 s |
| 统计窗口 | 3665 帧 |

结论：整体基本跟住了 10 Hz 点云输入。存在最大 319 ms 的输出间隔，说明个别帧处理或调度有抖动，但没有长期掉频。

### 4.2 CPU

全量采样，包括启动和结束阶段：

| 指标 | 数值 |
| --- | --- |
| 样本数 | 392 |
| 平均 CPU | 242.71% |
| 用户态平均 | 210.05% |
| 系统态平均 | 32.65% |
| 峰值 CPU | 780.00% |

运行中采样，过滤掉启动/结束阶段的 0% 空闲样本：

| 指标 | 数值 |
| --- | --- |
| 样本数 | 381 |
| 平均 CPU | 249.71% |
| 用户态平均 | 216.12% |
| 系统态平均 | 33.59% |
| 峰值 CPU | 780.00% |

解释：

- 平均 CPU 约 250%，即平均使用约 2.5 个 CPU core。
- 峰值 780% 表示某些瞬间会用到接近 8 个 core，主要来自 TBB 并行对应点搜索和线性系统构建。
- 系统态 CPU 约 34%，说明除纯计算外，点云转换、内存分配、ROS 消息处理也有一定开销。

### 4.3 内存

全量采样：

| 指标 | 数值 |
| --- | --- |
| 平均 RSS | 110264 KB，约 107.7 MB |
| 最小 RSS | 23616 KB，约 23.1 MB |
| 最大 RSS | 140084 KB，约 136.8 MB |
| 平均 VSZ | 2733445 KB，约 2.61 GB |
| 最大 VSZ | 2901660 KB，约 2.77 GB |

运行中采样，过滤掉启动初期低 RSS：

| 指标 | 数值 |
| --- | --- |
| 平均 RSS | 113237 KB，约 110.6 MB |
| 最小 RSS | 65232 KB，约 63.7 MB |
| 最大 RSS | 140084 KB，约 136.8 MB |

解释：

- 实际物理内存 RSS 峰值约 137 MB，不高。
- VSZ 约 2.7 GB，但这是虚拟地址空间，不等于真实占用；TBB、ROS、动态库和内存分配器都会扩大 VSZ。
- RSS 从前期约 90 MB 增长到后期约 140 MB，符合 `local_map` 随路线积累后进入滚动清理的行为，没有看到失控增长。

### 4.4 IO

`loc_lo_node` 自身 IO：

| 指标 | 数值 |
| --- | --- |
| 平均读 | 0.80 KB/s |
| 平均写 | 0.00 KB/s |
| 峰值读 | 260.00 KB/s |
| 峰值写 | 0.00 KB/s |

解释：

- 节点本身几乎没有磁盘 IO。
- 大量 IO 来自 `rosbag play` 读取 8.4 GB bag，不属于 `loc_lo_node` 本体开销。

`rosbag play` 进程资源：

| 指标 | 数值 |
| --- | --- |
| 墙钟时间 | 6:10.71 |
| CPU | 8% |
| 最大 RSS | 55980 KB，约 54.7 MB |
| File system inputs | 16911680 blocks |
| File system outputs | 34912 blocks |

## 5. 运行现象

1. 首帧 INS 初始化成功：

```text
LocLO initialized from INS pose xyz=228.174 -48.0891 -0.0468386
```

2. 点云处理过程中 `/genz/odometry` 基本稳定在 10 Hz 附近。

3. 虽然 launch 中设置了 `visualize:=false`，底层 `Registration` 仍然在终端持续输出 GenZ-ICP 彩色状态条。这些状态条没有大量写入 ROS 日志文件，但会刷 roslaunch 控制台，影响人工查看，也会带来少量 stdout 开销。

4. ROS 日志目录本次只有约 60 KB，没有出现日志文件爆炸。

## 6. 结论

在关闭 RViz、调试点云、全局地图和录包的纯算法配置下，`loc_lo_node` 可以完整处理 368.9 秒、10 Hz 点云数据。

资源结论：

- CPU：平均约 2.5 核，瞬时峰值接近 8 核。
- 内存：RSS 峰值约 137 MB，整体较轻。
- IO：节点本身几乎无磁盘 IO。
- 实时性：`/genz/odometry` 平均 9.935 Hz，基本跟住输入点云。

总体判断：当前配置下资源占用主要瓶颈是 CPU，不是内存或磁盘。CPU 峰值来自 ICP 对应点搜索、TBB 并行构建线性系统、体素地图更新等计算环节。

## 7. 建议

### 7.1 短期建议

1. 关闭底层终端状态条。

当前 `visualize:=false` 并不会关闭 `Registration` 的终端状态输出。建议在 `Loc_LO.cpp` 构造 `odometry_` 后增加：

```cpp
odometry_.SetTerminalStatusEnabled(false);
```

这样资源测试和日常运行日志会更干净。

2. 保持资源测试配置分层。

建议后续继续按下面四组测：

```text
纯算法：rviz=false visualize=false publish_global_map=false record=false
加录包：rviz=false visualize=false publish_global_map=false record=true
加 debug cloud：rviz=false visualize=true publish_global_map=false record=false
全可视化：rviz=true visualize=true publish_global_map=true record=false
```

3. 录包评估时仍建议只录：

```text
/genz/odometry
/localization/ins
```

不要录点云、local_map、global_map。

### 7.2 算法资源优化方向

如果后续 CPU 需要下降，可以优先尝试：

- 降低 `desired_num_voxelized_points`，例如从 3000 测到 2000、1500。
- 增大 `voxel_size`，例如从 0.6 测到 0.8。
- 降低 `max_num_iterations`，例如从 100 测到 60。
- 降低调试点云发布频率，尤其是 `/genz/local_map`。
- 给 `Registration::RegisterFrame()` 增加每帧耗时和对应点数量统计，定位 CPU 峰值对应的场景。

### 7.3 稳定性观察点

本次只做资源测试，没有做轨迹精度评估。下一步建议结合 evo 对同一次或新一次录包进行：

```bash
evo_ape bag /home/guoli/data/yangpu/loc/loc_lo_eval.bag \
  /localization/ins /genz/odometry \
  -r trans_part --align --project_to_plane xy --t_max_diff 0.05 --plot
```

资源和精度要一起看：例如降低点数能省 CPU，但可能导致长直路、开阔区域或动态物体多的场景漂移变大。
