# YangpuMapLocalization.cpp 流程说明

这份文档专门说明 `ros/tools/YangpuMapLocalization.cpp` 的流程。它是为杨浦数据集写的地图定位验证工具，核心目标是：把实时 LiDAR 点云配准到已有 PCD 地图上，输出车辆在 `map` 坐标系下的定位结果，并用 `/localization/ins` 做初始化、轨迹对比和误差评估。

对应文件：

- `ros/tools/YangpuMapLocalization.cpp`
- `ros/launch/yangpu_map_localization.launch`
- `ros/rviz/yangpu_map_localization.rviz`

## 1. 一句话理解

`YangpuMapLocalization.cpp` 做的是 scan-to-global-map localization：

```text
输入：
  1. 全局 PCD 地图
  2. /localization/ins
  3. /lidar_preprocessor/meta_cloud

处理：
  点云预处理 -> 用 INS/上一帧 ICP 给初值 -> GenZ-ICP scan-to-map 配准

输出：
  /yangpu_genz/odometry
  /yangpu_genz/trajectory
  /yangpu_genz/ins_trajectory
  /yangpu_genz/global_map
  /yangpu_genz/aligned_scan
  /yangpu_genz/planar_points
  /yangpu_genz/non_planar_points
  CSV 误差文件
```

它和原始 `OdometryServer.cpp` 的主要区别是：`OdometryServer.cpp` 是连续点云里程计，会维护局部地图；`YangpuMapLocalization.cpp` 是拿外部 PCD 当固定全局地图，每帧点云都对齐到这张地图。

## 2. 总体流程图

```mermaid
flowchart TD
    A["启动 yangpu_map_localization"] --> B["ParseArgs<br/>解析 launch/命令行参数"]
    B --> C{是否 --online}

    C -- 是 --> D["OnlineMapLocalizer 构造<br/>创建 publisher/subscriber<br/>打开 CSV"]
    D --> E["等待 /localization/ins"]
    E --> F["第一帧 INS 到来"]
    F --> G["InitializeMapFromIns"]
    G --> H["LoadMap<br/>读取 PCD + 半径裁剪 + 体素化"]
    H --> I["发布 /yangpu_genz/global_map"]
    I --> J["等待 /lidar_preprocessor/meta_cloud"]
    J --> K["CloudCallback<br/>单帧点云配准"]
    K --> J

    C -- 否 --> L["离线模式<br/>ExpandGlob 找 bag"]
    L --> M["FindFirstIns<br/>从 bag 中找第一帧 INS"]
    M --> N["LoadMap<br/>读取 PCD + 半径裁剪 + 体素化"]
    N --> O["rosbag::View<br/>按时间遍历 INS 和点云"]
    O --> P["点云帧配准"]
    P --> O
```

当前推荐使用在线模式，也就是通过 `yangpu_map_localization.launch` 启动节点和 RViz，然后你自己在另一个终端手动 `rosbag play`。

## 3. 在线模式流程

在线模式由 `--online` 触发，launch 文件默认就是这个模式。

### 3.1 启动阶段

启动命令通常是：

```bash
source /opt/ros/noetic/setup.bash
source /home/guoli/proj/genz-icp_ws/devel/setup.bash
roslaunch genz_icp yangpu_map_localization.launch
```

launch 会启动：

- `yangpu_map_localization` 节点
- RViz
- 可选 rosbag record，用于录制评估 bag

注意：launch 不会自动播放 bag。bag 需要你手动播放：

```bash
rosbag play /home/guoli/data/yangpu/loc/merged_bag.bag --clock
```

### 3.2 构造 `OnlineMapLocalizer`

`OnlineMapLocalizer` 构造函数做这些事：

1. 创建 `Registration` 配准器。
2. 打开 CSV 文件。
3. 创建输出话题 publisher。
4. 订阅 INS 和点云话题。
5. 等待第一帧 INS。

创建的话题包括：

| 话题 | 类型 | 作用 |
| --- | --- | --- |
| `/yangpu_genz/global_map` | `sensor_msgs/PointCloud2` | 发布裁剪/体素化后的地图 |
| `/yangpu_genz/aligned_scan` | `sensor_msgs/PointCloud2` | 发布配准到地图坐标系后的当前帧点云 |
| `/yangpu_genz/planar_points` | `sensor_msgs/PointCloud2` | 发布参与点到面约束的平面点 |
| `/yangpu_genz/non_planar_points` | `sensor_msgs/PointCloud2` | 发布参与点到点约束的非平面点 |
| `/yangpu_genz/odometry` | `nav_msgs/Odometry` | 发布 GenZ-ICP 地图定位结果 |
| `/yangpu_genz/trajectory` | `nav_msgs/Path` | 发布 GenZ-ICP 累计轨迹 |
| `/yangpu_genz/ins_trajectory` | `nav_msgs/Path` | 发布 INS 参考轨迹 |

## 4. INS 回调流程

INS 回调函数是 `InsCallback()`。

```mermaid
flowchart TD
    A["收到 /localization/ins"] --> B["OdomToSophus<br/>ROS Odometry 转 SE3"]
    B --> C["更新 latest_ins_"]
    C --> D["追加到 ins_path_msg_"]
    D --> E["发布 /yangpu_genz/ins_trajectory"]
    E --> F{是否第一帧 INS}
    F -- 否 --> G["结束<br/>等待下一帧消息"]
    F -- 是 --> H["打印初始 xyz/yaw"]
    H --> I["InitializeMapFromIns"]
    I --> J["LoadMap"]
    J --> K["发布 /yangpu_genz/global_map"]
```

第一帧 INS 非常关键，因为它有两个作用：

1. 作为地图裁剪中心。
2. 作为第一帧 ICP 的初始位姿。

如果第一帧 INS 和地图不是同一个坐标系，后续 ICP 初值会不准，地图裁剪也可能裁错区域。

## 5. 地图加载流程

地图加载函数是 `LoadMap()`。

```mermaid
flowchart TD
    A["LoadMap(options, first_ins)"] --> B["pcl::io::loadPCDFile<br/>读取 PCD"]
    B --> C["创建 VoxelHashMap"]
    C --> D["遍历 PCD 点"]
    D --> E{点是否 NaN/Inf}
    E -- 是 --> D
    E -- 否 --> F{map_radius > 0<br/>且超出首帧 INS 半径}
    F -- 是 --> D
    F -- 否 --> G["加入 chunk"]
    G --> H{chunk 是否满}
    H -- 是 --> I["map.AddPoints(chunk)<br/>写入体素哈希地图"]
    H -- 否 --> D
    I --> D
    D --> J["写入最后一批 chunk"]
    J --> K["打印 kept 点数和 voxel map 点数"]
    K --> L["返回 VoxelHashMap"]
```

关键参数：

| 参数 | 含义 | 影响 |
| --- | --- | --- |
| `map_path` | PCD 地图路径 | 决定使用哪张地图 |
| `map_radius` | 以首帧 INS 为圆心裁剪地图的半径 | 太小会开出地图范围，太大内存和计算量增加 |
| `map_voxel_size` | 地图体素大小 | 越小地图越密，越慢；越大地图越稀，细节减少 |
| `max_points_per_voxel` | 每个体素最多保留点数 | 控制地图密度 |
| `planarity_threshold` | 平面判断阈值 | 影响平面点/非平面点分类 |

注意：`pcl::io::loadPCDFile` 会先把整张 PCD 读入内存，然后代码才按 `map_radius` 裁剪。所以特别大的 PCD 最好提前离线降采样或裁剪。

## 6. 点云回调流程

点云回调函数是 `CloudCallback()`，这是在线定位的核心。

```mermaid
flowchart TD
    A["收到 /lidar_preprocessor/meta_cloud"] --> B{latest_ins_ 是否存在}
    B -- 否 --> C["等待 INS<br/>不处理点云"]
    B -- 是 --> D{global_map_ 是否存在}
    D -- 否 --> E["等待地图初始化<br/>不处理点云"]
    D -- 是 --> F["计算 elapsed<br/>检查 duration"]
    F --> G["PointCloud2ToEigen<br/>ROS 点云转 Eigen 点"]
    G --> H["Preprocess<br/>按 min/max range 裁剪"]
    H --> I["VoxelDownsample<br/>当前帧体素降采样"]
    I --> J["生成 initial_guess"]
    J --> K["Registration::RegisterFrame<br/>scan-to-map ICP"]
    K --> L["发布 odometry/path/aligned_scan"]
    L --> M["发布 planar/non_planar debug points"]
    M --> N["计算 ICP vs INS 误差"]
    N --> O["写 CSV"]
    O --> P["保存 previous_icp_ / previous_ins_"]
```

### 6.1 初值策略

初值生成逻辑如下：

```mermaid
flowchart TD
    A["准备 initial_guess"] --> B["默认使用当前 latest_ins_"]
    B --> C{是否已有 previous_icp_}
    C -- 否 --> D["第一帧 ICP<br/>使用当前 INS"]
    C -- 是 --> E["使用上一帧 ICP 位姿"]
    E --> F{use_ins_prediction<br/>且 previous_ins_ 存在}
    F -- 否 --> G["初值 = previous_icp_"]
    F -- 是 --> H["delta_ins = previous_ins^-1 * latest_ins"]
    H --> I["初值 = previous_icp_ * delta_ins"]
```

当前 launch 中使用了 `--no-ins-prediction`，也就是：

- 第一帧使用 INS。
- 后续默认使用上一帧 ICP。
- 不叠加 INS 帧间增量。

这样做的好处是评估时更能看出 ICP 自身是否稳定；坏处是如果某一帧 ICP 跳了，后续可能跟着偏。

### 6.2 配准调用

核心调用是：

```cpp
registration_.RegisterFrame(source,
                            *global_map_,
                            initial_guess,
                            options_.max_correspondence_distance,
                            options_.kernel);
```

输入含义：

- `source`：当前帧点云，经过距离裁剪和体素降采样。
- `global_map_`：从 PCD 构建的 `VoxelHashMap`。
- `initial_guess`：当前帧车体在 `map` 下的初始位姿。
- `max_correspondence_distance`：最大对应点距离。
- `kernel`：鲁棒核参数。

输出含义：

- `pose`：优化后的当前帧位姿。
- `planar_points`：被判定为平面约束的 source 点。
- `non_planar_points`：被判定为非平面约束的 source 点。

`planar_points` 和 `non_planar_points` 不是原始整帧点云，而是成功找到地图对应关系并参与优化的调试点。它们可以用来判断这一帧 ICP 有没有足够的几何约束。

## 7. 发布与记录

单帧 ICP 完成后会发布：

| 输出 | 说明 |
| --- | --- |
| `/yangpu_genz/odometry` | 当前帧 ICP 位姿，evo 评估主要看它 |
| `/yangpu_genz/trajectory` | ICP 累计轨迹，RViz 中红色 |
| `/yangpu_genz/ins_trajectory` | INS 累计轨迹，RViz 中黄色 |
| `/yangpu_genz/aligned_scan` | 用 ICP pose 变换到 map 下的当前帧点云 |
| `/yangpu_genz/planar_points` | 平面约束点，RViz 中绿色 |
| `/yangpu_genz/non_planar_points` | 非平面约束点，RViz 中橙色 |

CSV 每帧写一行：

| 字段 | 含义 |
| --- | --- |
| `stamp` | 当前点云时间戳 |
| `elapsed` | 相对第一帧点云经过的时间 |
| `frame` | 点云帧号 |
| `scan_points` | 原始点云转换后的点数 |
| `source_points` | 裁剪和降采样后的点数 |
| `icp_x/icp_y/icp_z/icp_yaw` | ICP 输出位姿 |
| `ins_x/ins_y/ins_z/ins_yaw` | 当前缓存的 INS 位姿 |
| `error_xy` | ICP 和 INS 的 XY 平面误差 |
| `error_z` | ICP 和 INS 的 Z 误差 |
| `error_yaw_rad` | ICP 和 INS 的 yaw 误差 |

## 8. 离线模式流程

如果不传 `--online`，程序会进入离线模式。离线模式不需要 `rosbag play`，它自己打开 bag 文件。

```mermaid
flowchart TD
    A["main 非 --online"] --> B["ExpandGlob<br/>展开 bag 路径"]
    B --> C{是否找到 bag}
    C -- 否 --> D["退出，返回错误码 2"]
    C -- 是 --> E["FindFirstIns<br/>查第一帧 INS"]
    E --> F{是否找到 INS}
    F -- 否 --> G["退出，返回错误码 3"]
    F -- 是 --> H["LoadMap"]
    H --> I["创建 Registration"]
    I --> J["可选创建 publisher"]
    J --> K["打开 CSV"]
    K --> L["rosbag::View 查询 INS + 点云"]
    L --> M["按时间顺序遍历消息"]
    M --> N{消息类型}
    N -- INS --> O["更新 latest_ins"]
    O --> M
    N -- 点云 --> P["如果已有 latest_ins<br/>执行 scan-to-map ICP"]
    P --> Q["可选发布 RViz 话题"]
    Q --> R["写 CSV 和统计误差"]
    R --> M
```

离线模式适合快速复现实验、批量跑参数、只生成 CSV。但你现在主要使用的是在线模式，因为它更方便和 RViz、手动播放 bag 配合。

## 9. 和 OdometryServer.cpp 的关系

| 对比项 | YangpuMapLocalization.cpp | OdometryServer.cpp |
| --- | --- | --- |
| 任务 | 全局地图定位 | LiDAR 里程计 |
| 地图 | 外部 PCD 固定地图 | 在线维护局部地图 |
| 初值 | 第一帧 INS，后续上一帧 ICP 或 INS prediction | 内部运动模型 |
| 输出坐标系 | `map` | 通常是 `odom` |
| 是否会累计漂移 | 如果地图有效，理论上不应长期累计漂移 | 会随着里程计运行累计漂移 |
| 主要评估方式 | 和 `/localization/ins` 用 evo/CSV 对比 | 看相对轨迹和局部一致性 |
| 调试点云 | `/yangpu_genz/planar_points`、`/yangpu_genz/non_planar_points` | `/genz/planar_points`、`/genz/non_planar_points` |

简单说：`OdometryServer.cpp` 是原项目的通用里程计入口；`YangpuMapLocalization.cpp` 是为杨浦数据集和全局 PCD 地图验证定制的工具。

## 10. 常见问题与排查

### 10.1 RViz 一开始看不到地图

正常。在线模式下地图要等第一帧 `/localization/ins` 到来后才加载，因为代码要用第一帧 INS 决定地图裁剪中心。

检查：

```bash
rostopic echo -n 1 /localization/ins
rostopic echo -n 1 /yangpu_genz/global_map
```

### 10.2 地图像一个圆圈，车辆开出去后漂移

这是 `map_radius` 裁剪造成的。只加载了首帧 INS 周围一圈地图，车开出这个区域后就没有地图点可匹配。

处理方式：

- 增大 `map_radius`
- 或设置 `map_radius:=0.0` 使用完整地图
- 或提前制作覆盖完整路线的降采样 PCD

示例：

```bash
roslaunch genz_icp yangpu_map_localization.launch map_radius:=0.0
```

### 10.3 两条轨迹整体有固定偏移

先确认 `/localization/ins` 和 PCD 地图是否真在同一个 `map` 坐标系。如果不是，CSV 和 RViz 中的误差会整体偏大，evo 评估也需要先做坐标对齐。

### 10.4 平面点/非平面点很少

可能原因：

- `max_corr` 太小，对应点找不到。
- `scan_voxel` 太大，当前帧点太稀。
- 地图和当前点云不重叠。
- 初值偏差太大。
- 地图局部几何退化，例如道路开阔区域约束少。

建议先看 RViz：

- 蓝色 aligned scan 是否贴近地图。
- 绿色 planar points 是否分布在墙面、地面边界等稳定结构附近。
- 橙色 non-planar points 是否覆盖杆、边缘、树干、路沿等结构。

### 10.5 evo 评估建议

录包时不要录点云，文件会很大。launch 默认只录：

- `/yangpu_genz/odometry`
- `/localization/ins`

常用 XY APE：

```bash
evo_ape bag /home/guoli/data/yangpu/loc/yangpu_loc_genz_icp_eval2.bag \
  /localization/ins /yangpu_genz/odometry \
  -r trans_part --project_to_plane xy --t_max_diff 0.05 --plot
```

轨迹对比：

```bash
evo_traj bag /home/guoli/data/yangpu/loc/yangpu_loc_genz_icp_eval2.bag \
  /localization/ins /yangpu_genz/odometry \
  --ref /localization/ins --plot
```

## 11. 推荐阅读顺序

如果你要快速掌握这份代码，建议按下面顺序看：

1. `Options`：先理解所有参数。
2. `main()`：看在线/离线模式怎么分流。
3. `OnlineMapLocalizer` 构造函数：看发布/订阅了哪些话题。
4. `InsCallback()`：看第一帧 INS 如何触发地图加载。
5. `LoadMap()`：看 PCD 如何变成 `VoxelHashMap`。
6. `CloudCallback()`：看单帧点云如何完成 scan-to-map。
7. `Registration::RegisterFrame()`：再进入底层 ICP 优化细节。

先掌握前 6 步，就已经足够调 Yangpu 数据集的大部分问题。
