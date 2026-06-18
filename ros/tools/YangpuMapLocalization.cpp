// MIT License
//
// Yangpu 数据集地图定位验证工具。
//
// 一句话理解：
//   这个程序把“当前 LiDAR 点云”配准到“已有 PCD 地图”上，得到车辆在 map 坐标系下的位置，
//   同时用 /localization/ins 作为初始位姿和参考真值，方便在 RViz/evo/CSV 中看效果。
//
// 两种运行模式：
//   1. 在线模式（--online）：
//      launch 启动节点后，用户自己 rosbag play。
//      节点订阅 /localization/ins 和 /lidar_preprocessor/meta_cloud。
//      收到第一帧 INS 后才加载地图，并用这帧 INS 作为初始位姿。
//      后续每帧点云做 scan-to-map ICP，并发布 /yangpu_genz/odometry、/trajectory、/aligned_scan。
//      同时把 INS 累积为 /yangpu_genz/ins_trajectory，方便 RViz 直接对比两条轨迹。
//      调试时还会发布 /yangpu_genz/planar_points 和 /yangpu_genz/non_planar_points。
//
//   2. 离线模式（不加 --online）：
//      程序自己打开 bag，先找第一帧 INS，再加载地图，然后顺序处理 bag 中的 INS 和点云。
//      主要用于不依赖 rosbag play 的快速复现实验和生成 CSV。
//
// 在线模式的数据流：
//   /localization/ins
//       -> InsCallback()
//       -> 第一帧 INS 调 InitializeMapFromIns()
//       -> LoadMap() 读取 PCD 并按第一帧 INS 附近裁剪地图
//
//   /lidar_preprocessor/meta_cloud
//       -> CloudCallback()
//       -> PointCloud2 转 Eigen 点
//       -> 按距离裁剪 + 体素降采样
//       -> 以上一帧 ICP 或 INS 作为初值
//       -> Registration::RegisterFrame() 做 scan-to-map
//       -> 发布 Odometry/Path/Aligned Scan/平面点/非平面点，并写 CSV
//   INS 回调会同步发布 /yangpu_genz/ins_trajectory 作为参考轨迹。
//
// 注意：
//   - RViz 中的地图 /yangpu_genz/global_map 是 latched 发布，但只有第一帧 INS 到来并成功 LoadMap 后才会发布。
//   - 如果 map_path 指向大 PCD，PCL 会先把整个 PCD 读进内存，再由 LoadMap 做半径裁剪。
//     因此大图建议提前降采样或拆分，否则可能启动时内存压力很大。

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <glob.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"
#include "ros1/Utils.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

// Options 是整个工具的统一配置入口。
// 命令行参数、launch 参数最终都会写入这个结构体。
// 后续在线/离线两条流程都只读 Options，避免参数散落在代码各处。
struct Options {
    // 离线模式使用的 bag 匹配表达式；在线模式不会读取它。
    std::string bag_glob = "/home/guoli/data/yangpu/A203/A203/2026-06-03-1*.bag";
    // 全局 PCD 地图路径。
    std::string map_path = "/home/guoli/data/yangpu/Map/global.pcd";
    // 输入点云话题；Yangpu 当前使用融合后的 base_link 点云。
    std::string cloud_topic = "/lidar_preprocessor/meta_cloud";
    // INS/组合导航里程计话题，用作第一帧初始化和评估参考。
    std::string ins_topic = "/localization/ins";
    // 输出 CSV 路径，记录每帧 ICP、INS 和误差。
    std::string output_csv = "/tmp/yangpu_genz_icp_validation.csv";
    // 运行时长限制；0 表示不限制，完整处理。
    double duration = 0.0;
    // 离线发布模式下的人为延时倍率；在线模式基本不用。
    double replay_rate = 1.0;
    // 地图裁剪半径；0 表示使用完整地图，大于 0 表示以初始位置为圆心裁剪。
    double map_radius = 160.0;
    // 点云最小距离，过滤车身附近过近点。
    double min_range = 0.5;
    // 点云最大距离，过滤远处点。
    double max_range = 100.0;
    // 地图体素大小，越大地图越稀疏、越快，但细节越少。
    double map_voxel_size = 0.6;
    // 当前帧点云体素大小，越大参与匹配点越少。
    double scan_voxel_size = 0.6;
    // ICP 最大对应点距离，超过该距离不建立匹配。
    double max_correspondence_distance = 2.0;
    // 鲁棒核参数，用于降低离群匹配影响。
    double kernel = 0.7;
    // 平面判断阈值，用于区分点到面约束和点到点约束。
    double planarity_threshold = 0.2;
    // 每个地图体素最多保留点数。
    int max_points_per_voxel = 3;
    // ICP 最大迭代次数。
    int max_iterations = 60;
    // ICP 收敛阈值。
    double convergence_criterion = 0.0001;
    // 是否用 INS 帧间增量辅助预测；false 表示只用第一帧 INS，后续靠上一帧 ICP。
    bool use_ins_prediction = true;
    // 是否发布 RViz/评估话题。
    bool publish = false;
    // 是否发布地图点云；完整地图较大时可关闭。
    bool publish_map = true;
    // 是否在线订阅话题；true 时不会读取 bag 文件。
    bool online = false;
    // 输出结果所在坐标系，默认 map。
    std::string frame_id = "map";
    // 兼容旧命令行保留的手写初值 x；当前在线 launch 已改为第一帧 INS 初始化，一般不用。
    double init_x = 298.88036951499384;
    // 兼容旧命令行保留的手写初值 y；当前在线 launch 已改为第一帧 INS 初始化，一般不用。
    double init_y = -41.43684999729909;
    // 兼容旧命令行保留的手写初值 z；当前在线 launch 已改为第一帧 INS 初始化，一般不用。
    double init_z = -0.041215605079837764;
    // 兼容旧命令行保留的手写初值 yaw；当前在线 launch 已改为第一帧 INS 初始化，一般不用。
    double init_yaw = -2.353;
};

// 打印命令行帮助。
// 当参数缺失、参数未知，或者用户传 --help 时会调用它。
void PrintUsage(const char *argv0) {
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "  --bag-glob PATH_GLOB        default: /home/guoli/data/yangpu/A203/A203/2026-06-03-1*.bag\n"
        << "  --map PATH                  default: /home/guoli/data/yangpu/Map/global.pcd\n"
        << "  --cloud-topic TOPIC         default: /lidar_preprocessor/meta_cloud\n"
        << "  --ins-topic TOPIC           default: /localization/ins\n"
        << "  --duration SEC              default: 0, run all data; positive value limits seconds\n"
        << "  --output CSV                default: /tmp/yangpu_genz_icp_validation.csv\n"
        << "  --publish                   publish map, aligned scans, odometry, and trajectory for RViz\n"
        << "  --online                    do not read bag files; subscribe to live/rosbag-play topics\n"
        << "  --replay-rate RATE          default: 1.0, only used with --publish\n"
        << "  --frame-id FRAME            default: map\n"
        << "  --init-x X                  legacy manual init x, online launch normally uses first INS\n"
        << "  --init-y Y                  legacy manual init y, online launch normally uses first INS\n"
        << "  --init-z Z                  legacy manual init z, online launch normally uses first INS\n"
        << "  --init-yaw RAD              legacy manual init yaw, online launch normally uses first INS\n"
        << "  --map-radius M              default: 160, crop global map around first INS pose\n"
        << "  --map-voxel M               default: 0.6\n"
        << "  --scan-voxel M              default: 0.6\n"
        << "  --max-corr M                default: 2.0\n"
        << "  --kernel VALUE              default: 0.7\n"
        << "  --no-ins-prediction         use previous ICP pose only after first-frame INS init\n";
}

bool ParseArgs(int argc, char **argv, Options *options) {
    // 从 argv[1] 开始逐个解析命令行参数。
    for (int i = 1; i < argc; ++i) {
        // 当前正在处理的参数名。
        const std::string arg = argv[i];
        // 小工具函数：要求当前参数后面必须跟一个值。
        auto need_value = [&](const std::string &name) -> std::string {
            // 如果没有后续值，说明用户命令行写错。
            if (i + 1 >= argc) {
                // 抛异常后外层 catch 会打印帮助信息。
                throw std::runtime_error("Missing value for " + name);
            }
            // 移动到下一个 argv，并返回它作为参数值。
            return argv[++i];
        };
        try {
            // 下面每个分支把命令行字符串写入 Options。
            if (arg == "--bag-glob") options->bag_glob = need_value(arg);
            else if (arg == "--map") options->map_path = need_value(arg);
            else if (arg == "--cloud-topic") options->cloud_topic = need_value(arg);
            else if (arg == "--ins-topic") options->ins_topic = need_value(arg);
            else if (arg == "--duration") options->duration = std::stod(need_value(arg));
            else if (arg == "--output") options->output_csv = need_value(arg);
            else if (arg == "--replay-rate") options->replay_rate = std::stod(need_value(arg));
            else if (arg == "--frame-id") options->frame_id = need_value(arg);
            else if (arg == "--init-x") options->init_x = std::stod(need_value(arg));
            else if (arg == "--init-y") options->init_y = std::stod(need_value(arg));
            else if (arg == "--init-z") options->init_z = std::stod(need_value(arg));
            else if (arg == "--init-yaw") options->init_yaw = std::stod(need_value(arg));
            else if (arg == "--map-radius") options->map_radius = std::stod(need_value(arg));
            else if (arg == "--map-voxel") options->map_voxel_size = std::stod(need_value(arg));
            else if (arg == "--scan-voxel") options->scan_voxel_size = std::stod(need_value(arg));
            else if (arg == "--max-corr") options->max_correspondence_distance = std::stod(need_value(arg));
            else if (arg == "--kernel") options->kernel = std::stod(need_value(arg));
            else if (arg == "--max-iterations") options->max_iterations = std::stoi(need_value(arg));
            else if (arg == "--no-ins-prediction") options->use_ins_prediction = false;
            else if (arg == "--publish") options->publish = true;
            else if (arg == "--no-publish-map") options->publish_map = false;
            else if (arg == "--online") options->online = true;
            else if (arg == "--help" || arg == "-h") {
                // 用户请求帮助时打印用法。
                PrintUsage(argv[0]);
                // 返回 false，让 main 退出。
                return false;
            } else {
                // 未知参数直接报错，避免静默忽略导致误配置。
                throw std::runtime_error("Unknown option: " + arg);
            }
        } catch (const std::exception &e) {
            // 打印具体错误原因。
            std::cerr << e.what() << "\n";
            // 再打印完整用法，方便用户修命令。
            PrintUsage(argv[0]);
            // 解析失败。
            return false;
        }
    }
    // 全部参数解析成功。
    return true;
}

// 展开 bag 路径通配符。
// 例如 /path/2026-06-03-1*.bag 会被展开成多个实际 bag 文件路径。
// 离线模式使用它；在线模式不会用，因为在线模式由用户自己 rosbag play。
std::vector<std::string> ExpandGlob(const std::string &pattern) {
    // glob_result 保存通配符展开结果。
    glob_t glob_result;
    // 使用系统 glob 展开路径，比如 2026-06-03-1*.bag。
    const int rc = glob(pattern.c_str(), GLOB_TILDE, nullptr, &glob_result);
    // 保存最终路径列表。
    std::vector<std::string> paths;
    // rc==0 表示 glob 成功匹配到文件。
    if (rc == 0) {
        // 预分配空间，减少 vector 扩容。
        paths.reserve(glob_result.gl_pathc);
        // 逐个拷贝匹配路径。
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            // 把 C 字符串路径放进 std::string。
            paths.emplace_back(glob_result.gl_pathv[i]);
        }
    }
    // 释放 glob 内部分配的内存。
    globfree(&glob_result);
    // 排序，保证多 bag 按名字顺序读取。
    std::sort(paths.begin(), paths.end());
    // 过滤 Windows 复制过来可能产生的 Zone.Identifier 伪文件。
    paths.erase(std::remove_if(paths.begin(), paths.end(), [](const std::string &path) {
                    // 只要路径里包含这个后缀，就认为不是 rosbag。
                    return path.find(":Zone.Identifier") != std::string::npos;
                }),
                // erase-remove 惯用法的末尾迭代器。
                paths.end());
    // 返回干净的 bag 文件列表。
    return paths;
}

// 把 ROS nav_msgs/Odometry 转成 Sophus::SE3d。
// Sophus 更适合做位姿乘法、逆、相对运动等 SE(3) 运算。
Sophus::SE3d OdomToSophus(const nav_msgs::Odometry &odom) {
    // 取出 ROS Odometry 的平移部分。
    const auto &p = odom.pose.pose.position;
    // 取出 ROS Odometry 的四元数部分。
    const auto &q = odom.pose.pose.orientation;
    // Eigen 四元数构造顺序是 w,x,y,z，ROS 消息字段顺序是 x,y,z,w。
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    // 归一化，避免数值误差导致 Sophus 构造异常。
    quat.normalize();
    // 返回 Sophus SE3 位姿。
    return Sophus::SE3d(quat, Eigen::Vector3d(p.x, p.y, p.z));
}

// 从 SE3 位姿中提取车辆平面航向角 yaw。
// 评估时主要看 xy 平面误差和 yaw 误差，所以单独封装。
double YawFromPose(const Sophus::SE3d &pose) {
    // 从 SE3 中取单位四元数。
    const Eigen::Quaterniond q(pose.so3().unit_quaternion());
    // 按标准公式从四元数计算 yaw。
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

// 把角度归一化到 [-pi, pi]。
// 否则两个实际很接近的角度，例如 +179 度和 -179 度，会被误认为差了 358 度。
double WrapAngle(double angle) {
    // 把角度归一化到 [-pi, pi]，避免 359 度和 -1 度被误认为差很多。
    while (angle > M_PI) angle -= 2.0 * M_PI;
    // 小于 -pi 时加 2pi。
    while (angle < -M_PI) angle += 2.0 * M_PI;
    // 返回归一化角度。
    return angle;
}

// 把 Sophus::SE3d 转成 ROS geometry_msgs::Pose。
// 发布 Odometry 和 Path 时需要 ROS 消息格式。
geometry_msgs::Pose SophusToPose(const Sophus::SE3d &pose) {
    // 创建 ROS Pose 消息。
    geometry_msgs::Pose msg;
    // 写入 x 平移。
    msg.position.x = pose.translation().x();
    // 写入 y 平移。
    msg.position.y = pose.translation().y();
    // 写入 z 平移。
    msg.position.z = pose.translation().z();
    // 从 Sophus 旋转取 Eigen 四元数。
    const Eigen::Quaterniond q(pose.so3().unit_quaternion());
    // 写入 ROS 四元数 x。
    msg.orientation.x = q.x();
    // 写入 ROS 四元数 y。
    msg.orientation.y = q.y();
    // 写入 ROS 四元数 z。
    msg.orientation.z = q.z();
    // 写入 ROS 四元数 w。
    msg.orientation.w = q.w();
    // 返回 ROS Pose。
    return msg;
}

// 用 xyz + yaw 构造一个 SE3 位姿。
// 当前主要作为旧版手写 init 参数的兼容工具函数。
Sophus::SE3d PoseFromXyzYaw(double x, double y, double z, double yaw) {
    // 用 yaw 构造绕 z 轴旋转的四元数，再和 xyz 组成 SE3。
    return Sophus::SE3d(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())),
                        Eigen::Vector3d(x, y, z));
}

// 读取 PCD 地图并构造成 GenZ-ICP 的 VoxelHashMap。
//
// 参数：
//   options.map_path       PCD 文件路径。
//   options.map_radius     地图裁剪半径；>0 时只保留 first_ins 周围圆形区域，=0 时使用整张 PCD。
//   options.map_voxel_size 地图体素大小。
//   first_ins              地图裁剪中心；在线模式来自第一帧 INS，离线模式来自 bag 第一帧 INS。
//
// 返回：
//   GenZ-ICP 内部使用的体素哈希地图，后续 ICP 会在这里找对应点。
//
// 注意：
//   PCL loadPCDFile 会先把 PCD 整体读到内存，之后本函数才裁剪。
//   因此如果原始 PCD 很大，最好先离线降采样/裁剪成更小的 PCD。
genz_icp::VoxelHashMap LoadMap(const Options &options, const Sophus::SE3d &first_ins) {
    // PCL 点云容器；地图包含 x/y/z/intensity。
    pcl::PointCloud<pcl::PointXYZI> cloud;
    // 打印地图路径，便于确认加载的是哪张地图。
    std::cout << "Loading PCD map: " << options.map_path << std::endl;
    // 用 PCL 读取 PCD 文件。
    if (pcl::io::loadPCDFile(options.map_path, cloud) != 0) {
        // 读取失败直接抛异常，节点无法继续定位。
        throw std::runtime_error("Failed to load PCD map: " + options.map_path);
    }

    // 构造 GenZ-ICP 使用的体素哈希地图。
    genz_icp::VoxelHashMap map(options.map_voxel_size,
                               options.max_range,
                               std::numeric_limits<double>::max(),
                               options.planarity_threshold,
                               options.max_points_per_voxel);
    // 地图裁剪中心：离线模式用 bag 首帧 INS，在线模式用订阅到的第一帧 INS。
    const Eigen::Vector3d origin = first_ins.translation();
    // 分批写入地图，避免一次性临时 vector 过大。
    constexpr size_t chunk_size = 200000;
    // 当前批次缓存。
    std::vector<Eigen::Vector3d> chunk;
    // 提前分配当前批次容量。
    chunk.reserve(chunk_size);
    // 统计裁剪后保留下来的原始地图点数量。
    size_t kept = 0;
    // 半径平方，避免循环里反复 sqrt。
    const double radius2 = options.map_radius * options.map_radius;

    // 遍历 PCD 中每个地图点。
    for (const auto &pt : cloud.points) {
        // 过滤 NaN/Inf 点。
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        // 转成 Eigen 三维点。
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        // 只用 xy 平面距离判断是否在裁剪圆内。
        const Eigen::Vector2d d = (p - origin).head<2>();
        // map_radius>0 时启用局部地图裁剪；map_radius=0 表示完整地图。
        if (options.map_radius > 0.0 && d.squaredNorm() > radius2) continue;
        // 把保留点放入当前批次。
        chunk.emplace_back(p);
        // 累加保留点计数。
        ++kept;
        // 当前批次满了就写入体素哈希地图。
        if (chunk.size() >= chunk_size) {
            // AddPoints 会按 map_voxel_size 建体素并限点。
            map.AddPoints(chunk);
            // 清空批次缓存，继续读后续地图点。
            chunk.clear();
        }
    }
    // 循环结束后，把最后不足一个批次的点写入地图。
    if (!chunk.empty()) map.AddPoints(chunk);
    // 打印原始保留点数和体素化后点数，用于估计内存和地图密度。
    std::cout << "Map points kept: " << kept << ", voxel map points: " << map.Pointcloud().size()
              << std::endl;
    // 返回体素哈希地图。
    return map;
}

// 离线模式专用：从一个或多个 bag 中找到第一帧 INS。
// 找到后返回 SE3 位姿，用于初始化地图裁剪中心和第一帧 ICP 初值。
std::optional<Sophus::SE3d> FindFirstIns(const std::vector<std::string> &bag_paths,
                                         const std::string &ins_topic) {
    std::vector<std::unique_ptr<rosbag::Bag>> bags;
    rosbag::View view;
    for (const auto &path : bag_paths) {
        auto bag = std::make_unique<rosbag::Bag>();
        bag->open(path, rosbag::bagmode::Read);
        view.addQuery(*bag, rosbag::TopicQuery({ins_topic}));
        bags.emplace_back(std::move(bag));
    }
    for (const auto &message : view) {
        const auto odom = message.instantiate<nav_msgs::Odometry>();
        if (odom) return OdomToSophus(*odom);
    }
    return std::nullopt;
}

class OnlineMapLocalizer {
public:
    // 在线定位类构造函数：创建发布器并订阅 INS/点云；地图等第一帧 INS 到来后再加载。
    OnlineMapLocalizer(ros::NodeHandle &nh, const Options &options)
        // 保存外部传入的 NodeHandle。
        : nh_(nh),
          // 保存命令行/launch 参数。
          options_(options),
          // 构造底层 ICP 配准器。
          registration_(options.max_iterations, options.convergence_criterion) {
        // 关闭 Registration 内部终端动画，避免刷屏影响 ROS 日志。
        registration_.SetTerminalStatusEnabled(false);

        // 打开 CSV 文件，用于后续离线分析误差。
        csv_.open(options.output_csv);
        // 如果 CSV 打不开，说明路径权限或目录有问题，直接报错。
        if (!csv_) {
            // 抛异常给 main 捕获并打印 ROS_ERROR。
            throw std::runtime_error("Failed to open output CSV: " + options.output_csv);
        }
        // 写 CSV 表头，后续每帧追加一行。
        csv_ << "stamp,elapsed,frame,scan_points,source_points,"
                "icp_x,icp_y,icp_z,icp_yaw,ins_x,ins_y,ins_z,ins_yaw,"
                "error_xy,error_z,error_yaw_rad\n";

        // 发布全局地图点云，latched=true，新打开 RViz 也能收到最后一次地图。
        map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/yangpu_genz/global_map", 1, true);
        // 发布每帧配准后的点云，用于 RViz 看 scan 是否贴合地图。
        aligned_scan_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/yangpu_genz/aligned_scan", 2);
        // 发布当前帧中被判定为平面的约束点，用于调试点到面约束。
        planar_points_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/yangpu_genz/planar_points", 2);
        // 发布当前帧中被判定为非平面的约束点，用于调试点到点约束。
        non_planar_points_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/yangpu_genz/non_planar_points", 2);
        // 发布 GenZ-ICP 地图定位结果，evo 主要评估这个话题。
        odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("/yangpu_genz/odometry", 20);
        // 发布累计轨迹给 RViz；评估时不要录这个话题，Path 会越来越大。
        path_publisher_ = nh_.advertise<nav_msgs::Path>("/yangpu_genz/trajectory", 2, true);
        // 发布 INS 累计轨迹，和 ICP 轨迹叠加显示，方便肉眼比较漂移和跳变。
        ins_path_publisher_ = nh_.advertise<nav_msgs::Path>("/yangpu_genz/ins_trajectory", 2, true);
        // 设置 Path 的坐标系。
        path_msg_.header.frame_id = options.frame_id;
        // INS Path 也放在 map 坐标系下；这里默认 /localization/ins 与地图同系。
        ins_path_msg_.header.frame_id = options.frame_id;

        // 订阅 INS 里程计；第一帧 INS 会作为初始位姿来源。
        ins_subscriber_ = nh_.subscribe(options.ins_topic, 200, &OnlineMapLocalizer::InsCallback, this);
        // 订阅融合 LiDAR 点云；每来一帧就做一次 scan-to-map。
        cloud_subscriber_ = nh_.subscribe(options.cloud_topic, 20, &OnlineMapLocalizer::CloudCallback, this);

        // 打印提示：节点已经准备好，用户可以手动 rosbag play。
        ROS_INFO_STREAM("Yangpu online localization is ready. Play bag manually, e.g. rosbag play 2026-06-03-1*.bag");
        // 打印等待的 INS 话题名。
        ROS_INFO_STREAM("Waiting for INS topic: " << options.ins_topic);
        // 打印等待的点云话题名。
        ROS_INFO_STREAM("Waiting for cloud topic: " << options.cloud_topic);
        // 提示在线模式的初始化方式。
        ROS_INFO_STREAM("Map will be loaded after the first INS pose is received");
    }

private:
    // 在线模式地图初始化入口。
    //
    // 调用时机：
    //   只在 InsCallback 第一次收到 /localization/ins 时调用一次。
    //
    // 做的事情：
    //   1. 使用第一帧 INS 作为中心调用 LoadMap()。
    //   2. 把得到的 VoxelHashMap 存入 global_map_。
    //   3. 如果开启 publish_map，把地图发布到 /yangpu_genz/global_map。
    //
    // 为什么不在构造函数加载地图：
    //   构造函数执行时还没收到 loc/A203 等不同数据集的真实第一帧 INS。
    //   如果提前用手写 init_x/init_y，换数据集时很容易裁错地图区域。
    void InitializeMapFromIns(const Sophus::SE3d &first_ins) {
        try {
            // 用第一帧 INS 作为地图裁剪中心，避免使用旧数据集的手写 init_x/init_y。
            global_map_.emplace(LoadMap(options_, first_ins));
        } catch (const std::exception &e) {
            ROS_ERROR_STREAM("Failed to initialize map from first INS pose: " << e.what());
            ros::shutdown();
            return;
        }

        // 根据参数决定是否发布地图；完整地图很大，调试时可关闭。
        if (options_.publish_map) {
            // 创建地图点云 header。
            std_msgs::Header header;
            // 地图是静态数据，时间戳用当前 ROS 时间即可。
            header.stamp = ros::Time::now();
            // 地图坐标系通常是 map。
            header.frame_id = options_.frame_id;
            // 把体素哈希地图转回点云并发布给 RViz。
            map_publisher_.publish(*genz_icp_ros::utils::EigenToPointCloud2(global_map_->Pointcloud(), header));
        }

        ROS_INFO_STREAM("Map initialized from first INS pose");
    }

    // INS 回调：缓存最新 INS 位姿，并在第一帧 INS 到来时初始化地图。
    //
    // latest_ins_ 的用途有两个：
    //   1. 第一帧：作为地图裁剪中心和第一帧 ICP 初值。
    //   2. 后续帧：作为误差评估参考；如果 use_ins_prediction=true，也用于提供帧间运动预测。
    void InsCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        // 把 ROS Odometry 转成 Sophus SE3，便于后续和 ICP 位姿做运算。
        latest_ins_ = OdomToSophus(*msg);
        // 把每帧 INS 也累积成 Path，RViz 中可直接和 /yangpu_genz/trajectory 对比。
        geometry_msgs::PoseStamped ins_pose_msg;
        // INS 轨迹使用原始 INS 时间戳，但坐标系统一显示为 map。
        ins_pose_msg.header.stamp = msg->header.stamp;
        // 这里假设 /localization/ins 已经在 map/global 坐标系；如果不是，需要先做坐标变换。
        ins_pose_msg.header.frame_id = options_.frame_id;
        // 位姿直接使用 INS odometry 中的 pose。
        ins_pose_msg.pose = msg->pose.pose;
        // 更新 Path header 时间戳。
        ins_path_msg_.header.stamp = msg->header.stamp;
        // 追加 INS 轨迹点。
        ins_path_msg_.poses.push_back(ins_pose_msg);
        // 发布 /yangpu_genz/ins_trajectory。
        ins_path_publisher_.publish(ins_path_msg_);
        // 只在第一次收到 INS 时打印初始化信息。
        if (!first_ins_received_) {
            // 标记已经收到第一帧 INS。
            first_ins_received_ = true;
            // 打印第一帧 INS 位姿，方便确认初值是否正常。
            ROS_INFO_STREAM("Received first INS init xyz: " << latest_ins_->translation().transpose()
                            << ", yaw: " << YawFromPose(*latest_ins_));
            // 在线模式真正用第一帧 INS 作为地图裁剪/初始化中心。
            InitializeMapFromIns(*latest_ins_);
        }
    }

    // 发布当前帧 ICP 的调试点云。
    //
    // planar_points 和 non_planar_points 来自 Registration::RegisterFrame() 最后一轮对应点搜索。
    // 它们不是整帧原始点云，而是“成功在地图中找到对应点，并参与优化”的 source 点。
    //
    // 坐标系说明：
    //   Registration 内部会先用 initial_guess 把 source 变到 map 下，再迭代更新 source。
    //   因此这里直接以 options_.frame_id 发布，方便和 /yangpu_genz/global_map 叠加观察。
    void PublishDebugPoints(const ros::Time &stamp,
                            const std::vector<Eigen::Vector3d> &planar_points,
                            const std::vector<Eigen::Vector3d> &non_planar_points) {
        // 构造调试点云 header。
        std_msgs::Header header;
        // 使用当前点云时间戳，方便 RViz 时间同步。
        header.stamp = stamp;
        // 调试点已经在地图坐标系附近。
        header.frame_id = options_.frame_id;
        // 发布平面约束点。
        planar_points_publisher_.publish(*genz_icp_ros::utils::EigenToPointCloud2(planar_points, header));
        // 发布非平面约束点。
        non_planar_points_publisher_.publish(*genz_icp_ros::utils::EigenToPointCloud2(non_planar_points, header));
    }

    // 点云回调：在线定位最核心的入口。
    //
    // 每来一帧 /lidar_preprocessor/meta_cloud，执行下面流程：
    //   1. 确认 INS 和地图都已准备好。
    //   2. PointCloud2 -> Eigen 点数组。
    //   3. 距离裁剪，去掉太近/太远的点。
    //   4. 体素降采样，减少 ICP 计算量。
    //   5. 生成 ICP 初值：
    //      - 第一帧用当前 INS；
    //      - 后续默认用上一帧 ICP；
    //      - 如果开启 INS prediction，则在上一帧 ICP 上叠加 INS 帧间增量。
    //   6. 调 Registration::RegisterFrame() 做 scan-to-map。
    //   7. 发布 odometry/path/aligned_scan/平面点/非平面点，并写一行 CSV。
    void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        // 没有 INS 初值时，不处理点云，避免 ICP 从错误初值开始。
        if (!latest_ins_) {
            // 2 秒最多打印一次警告，避免刷屏。
            ROS_WARN_THROTTLE(2.0, "Waiting for /localization/ins before processing point clouds");
            // 直接返回，等待下一帧点云。
            return;
        }
        // 地图尚未从第一帧 INS 初始化完成时，暂不处理点云。
        if (!global_map_) {
            // 2 秒最多打印一次警告，避免刷屏。
            ROS_WARN_THROTTLE(2.0, "Waiting for map initialization from first INS pose");
            // 直接返回，等待地图加载完成。
            return;
        }
        // 第一帧点云到来时，记录起始时间。
        if (!start_stamp_) start_stamp_ = msg->header.stamp;
        // 计算当前点云相对第一帧的运行时间。
        const double elapsed = (msg->header.stamp - *start_stamp_).toSec();
        // 如果设置了 duration 限制，超过指定时间后不再处理。
        if (options_.duration > 0.0 && elapsed > options_.duration) return;

        // 把 ROS PointCloud2 转成 Eigen 三维点数组。
        const auto raw_points = genz_icp_ros::utils::PointCloud2ToEigen(msg);
        // 按距离裁剪点云，去掉过近和过远点。
        const auto cropped = genz_icp::Preprocess(raw_points, options_.max_range, options_.min_range);
        // 对当前帧点云体素下采样，减少 ICP 计算量。
        const auto source = genz_icp::VoxelDownsample(cropped, options_.scan_voxel_size);

        // 默认初值使用当前 INS；第一帧 ICP 就会走这里。
        Sophus::SE3d initial_guess = *latest_ins_;
        // 如果已经有上一帧 ICP 结果，后续优先用上一帧 ICP 做初值。
        if (previous_icp_) {
            // 将初值设为上一帧 ICP 位姿，适合连续低速运动。
            initial_guess = *previous_icp_;
            // 如果启用 INS prediction，就把 INS 帧间运动叠加到上一帧 ICP 上。
            if (options_.use_ins_prediction && previous_ins_) {
                // 计算上一帧 INS 到当前 INS 的相对运动。
                const Sophus::SE3d delta_ins = previous_ins_->inverse() * (*latest_ins_);
                // 用上一帧 ICP 位姿加 INS 增量作为当前初值。
                initial_guess = (*previous_icp_) * delta_ins;
            }
        }

        // 调用 GenZ-ICP 底层配准：把当前帧 source 对齐到全局体素地图。
        //
        // RegisterFrame 的输入：
        //   source：当前帧降采样后的点云，仍在车体/点云局部坐标系下。
        //   *global_map_：PCD 地图构成的 VoxelHashMap，已经在 map 坐标系下。
        //   initial_guess：当前车体在 map 下的初值。
        //
        // RegisterFrame 的输出：
        //   pose：优化后的车体位姿，map -> base_link 的含义。
        //   planar_points / non_planar_points：参与匹配的平面/非平面约束点，用于统计 corr。
        const auto [pose, planar_points, non_planar_points] =
            registration_.RegisterFrame(source,
                                        *global_map_,
                                        initial_guess,
                                        options_.max_correspondence_distance,
                                        options_.kernel);

        // 构造输出消息 header。
        std_msgs::Header header;
        // 在线模式使用输入点云时间戳，便于和 INS/evo 对齐。
        header.stamp = msg->header.stamp;
        // 输出坐标系通常是 map。
        header.frame_id = options_.frame_id;

        // 构造 Odometry 输出。
        nav_msgs::Odometry odom_msg;
        // 写入时间戳和坐标系。
        odom_msg.header = header;
        // 子坐标系写 base_link，表示输出的是车体位姿。
        odom_msg.child_frame_id = "base_link";
        // 写入 ICP 估计位姿。
        odom_msg.pose.pose = SophusToPose(pose);
        // 发布 /yangpu_genz/odometry。
        odom_publisher_.publish(odom_msg);

        // 构造当前帧轨迹点。
        geometry_msgs::PoseStamped pose_msg;
        // 轨迹点使用同一个 header。
        pose_msg.header = header;
        // 轨迹点位姿就是当前 odometry 位姿。
        pose_msg.pose = odom_msg.pose.pose;
        // 更新 Path header 的时间戳。
        path_msg_.header.stamp = header.stamp;
        // 把当前位姿追加到累计轨迹。
        path_msg_.poses.push_back(pose_msg);
        // 发布 /yangpu_genz/trajectory 给 RViz。
        path_publisher_.publish(path_msg_);
        // 把当前帧点云按 ICP 位姿变换到 map 下并发布，便于看是否贴合地图。
        aligned_scan_publisher_.publish(*genz_icp_ros::utils::EigenToPointCloud2(source, pose, header));
        // 发布参与本次 ICP 优化的平面点和非平面点，方便调试匹配质量。
        PublishDebugPoints(msg->header.stamp, planar_points, non_planar_points);

        // 计算 ICP 位姿与当前 INS 位置的差。
        // 这里假设 INS 和地图都在同一个 map 坐标系；如果存在固定坐标偏移，CSV 误差会整体偏大。
        const Eigen::Vector3d err = pose.translation() - latest_ins_->translation();
        // 只看 xy 平面误差，车辆定位通常更关心这个。
        const double xy_error = err.head<2>().norm();
        // 计算 yaw 误差，并归一化到 [-pi, pi]。
        const double yaw_error = WrapAngle(YawFromPose(pose) - YawFromPose(*latest_ins_));
        // 追加一行 CSV，方便后处理画图和统计精度。
        csv_ << std::fixed << std::setprecision(9)
             << msg->header.stamp.toSec() << "," << elapsed << "," << frame_index_ << ","
             << raw_points.size() << "," << source.size() << ","
             << pose.translation().x() << "," << pose.translation().y() << ","
             << pose.translation().z() << "," << YawFromPose(pose) << ","
             << latest_ins_->translation().x() << "," << latest_ins_->translation().y() << ","
             << latest_ins_->translation().z() << "," << YawFromPose(*latest_ins_) << ","
             << xy_error << "," << err.z() << "," << yaw_error << "\n";

        // 节流打印当前状态，1 秒最多一次，避免日志太多。
        ROS_INFO_STREAM_THROTTLE(1.0, "frame " << frame_index_
                                 << " elapsed=" << elapsed
                                 << " source=" << source.size()
                                 << " corr=" << (planar_points.size() + non_planar_points.size())
                                 << " xy_err=" << xy_error
                                 << " yaw_err=" << yaw_error);

        // 保存当前 ICP 位姿，下一帧用作初值。
        previous_icp_ = pose;
        // 保存当前 INS 位姿，下一帧如果启用 INS prediction 会用它计算增量。
        previous_ins_ = latest_ins_;
        // 帧计数加一。
        ++frame_index_;
    }

    // ROS NodeHandle，用于创建订阅器和发布器。
    ros::NodeHandle nh_;
    // 参数配置。
    Options options_;
    // 全局体素地图；在线模式收到第一帧 INS 后才构造。
    std::optional<genz_icp::VoxelHashMap> global_map_;
    // 底层 GenZ-ICP 配准器。
    genz_icp::Registration registration_;
    // 全局地图发布器。
    ros::Publisher map_publisher_;
    // 配准后当前帧点云发布器。
    ros::Publisher aligned_scan_publisher_;
    // 平面约束点发布器。
    ros::Publisher planar_points_publisher_;
    // 非平面约束点发布器。
    ros::Publisher non_planar_points_publisher_;
    // 里程计发布器。
    ros::Publisher odom_publisher_;
    // 轨迹发布器。
    ros::Publisher path_publisher_;
    // INS 轨迹发布器。
    ros::Publisher ins_path_publisher_;
    // INS 订阅器。
    ros::Subscriber ins_subscriber_;
    // 点云订阅器。
    ros::Subscriber cloud_subscriber_;
    // 累计轨迹缓存。
    nav_msgs::Path path_msg_;
    // INS 累计轨迹缓存。
    nav_msgs::Path ins_path_msg_;
    // CSV 输出文件。
    std::ofstream csv_;
    // 最新收到的 INS 位姿。
    std::optional<Sophus::SE3d> latest_ins_;
    // 上一帧 INS 位姿。
    std::optional<Sophus::SE3d> previous_ins_;
    // 上一帧 ICP 位姿。
    std::optional<Sophus::SE3d> previous_icp_;
    // 第一帧点云时间戳，用于计算 elapsed。
    std::optional<ros::Time> start_stamp_;
    // 是否已经收到第一帧 INS。
    bool first_ins_received_ = false;
    // 已处理点云帧数。
    size_t frame_index_ = 0;
};

}  // namespace

int main(int argc, char **argv) {
    // 初始化 ROS 节点，节点名为 yangpu_map_localization。
    ros::init(argc, argv, "yangpu_map_localization");
    // 创建参数结构体，后续由命令行覆盖默认值。
    Options options;
    // 解析命令行参数；失败时直接退出。
    if (!ParseArgs(argc, argv, &options)) return 1;
    // 创建 ROS NodeHandle，用于在线模式创建订阅/发布。
    ros::NodeHandle nh;

    // 如果传入 --online，则进入在线订阅模式，不读取 bag。
    //
    // 在线模式适合配合 roslaunch + 手动 rosbag play：
    //   终端 1：roslaunch genz_icp yangpu_map_localization.launch
    //   终端 2：rosbag play /home/guoli/data/yangpu/loc/merged_bag.bag --clock
    //
    // 注意：在线模式下地图要等第一帧 INS 到来后才加载，所以 RViz 一开始可能看不到地图。
    if (options.online) {
        try {
            // 构造在线定位对象：打开 CSV、创建 publisher/subscriber；地图在第一帧 INS 后加载。
            OnlineMapLocalizer localizer(nh, options);
            // 进入 ROS 回调循环，等待 INS 和点云。
            ros::spin();
            // ros::spin 正常结束时返回 0。
            return 0;
        } catch (const std::exception &e) {
            // 捕获地图加载、CSV 打开等异常，并打印到 ROS 日志。
            ROS_ERROR_STREAM(e.what());
            // 返回非零值，表示启动失败。
            return 6;
        }
    }

    // 走到这里说明没有 --online，进入离线模式。
    //
    // 离线模式和在线模式做的是同一件事：scan-to-map 配准。
    // 区别是：
    //   在线模式从 ROS topic 收消息；
    //   离线模式直接打开 bag 文件，从 rosbag::View 里按时间顺序读消息。
    //
    // 离线模式：展开 bag 路径通配符。
    const auto bag_paths = ExpandGlob(options.bag_glob);
    // 如果没有匹配到 bag，无法继续。
    if (bag_paths.empty()) {
        // 打印用户传入的 bag_glob，方便检查路径。
        std::cerr << "No bag files matched: " << options.bag_glob << std::endl;
        // 返回错误码 2。
        return 2;
    }
    // 打印匹配到的 bag 数量和第一包路径。
    std::cout << "Using " << bag_paths.size() << " bag files. First: " << bag_paths.front()
              << std::endl;

    // 离线模式先从 bag 中找第一帧 INS。
    // 这一步必须在 LoadMap 前完成，因为地图裁剪中心需要首帧 INS。
    const auto first_ins = FindFirstIns(bag_paths, options.ins_topic);
    // 如果没有 INS，无法给全局地图定位提供初值。
    if (!first_ins) {
        // 打印缺失的话题名。
        std::cerr << "No INS pose found on " << options.ins_topic << std::endl;
        // 返回错误码 3。
        return 3;
    }
    // 打印首帧 INS 位姿，确认初始化是否合理。
    std::cout << std::fixed << std::setprecision(3)
              << "First INS init xyz: " << first_ins->translation().transpose()
              << ", yaw: " << YawFromPose(*first_ins) << std::endl;

    // 根据首帧 INS 位置加载/裁剪全局地图。
    auto global_map = LoadMap(options, *first_ins);
    // 构造底层 ICP 配准器。
    genz_icp::Registration registration(options.max_iterations, options.convergence_criterion);
    // 关闭终端动画输出。
    registration.SetTerminalStatusEnabled(false);

    // 离线模式下可选发布 RViz 话题。
    ros::Publisher map_publisher;
    // 配准后当前帧点云发布器。
    ros::Publisher aligned_scan_publisher;
    // 平面约束点发布器。
    ros::Publisher planar_points_publisher;
    // 非平面约束点发布器。
    ros::Publisher non_planar_points_publisher;
    // 里程计发布器。
    ros::Publisher odom_publisher;
    // 轨迹发布器。
    ros::Publisher path_publisher;
    // 累计轨迹缓存。
    nav_msgs::Path path_msg;
    // 设置轨迹坐标系。
    path_msg.header.frame_id = options.frame_id;
    // 如果用户加了 --publish，就创建 publisher。
    if (options.publish) {
        // 发布全局地图。
        map_publisher = nh.advertise<sensor_msgs::PointCloud2>("/yangpu_genz/global_map", 1, true);
        // 发布配准后当前帧点云。
        aligned_scan_publisher = nh.advertise<sensor_msgs::PointCloud2>("/yangpu_genz/aligned_scan", 2);
        // 发布当前帧中被判定为平面的约束点。
        planar_points_publisher = nh.advertise<sensor_msgs::PointCloud2>("/yangpu_genz/planar_points", 2);
        // 发布当前帧中被判定为非平面的约束点。
        non_planar_points_publisher = nh.advertise<sensor_msgs::PointCloud2>("/yangpu_genz/non_planar_points", 2);
        // 发布 ICP 输出 odometry。
        odom_publisher = nh.advertise<nav_msgs::Odometry>("/yangpu_genz/odometry", 20);
        // 发布累计轨迹。
        path_publisher = nh.advertise<nav_msgs::Path>("/yangpu_genz/trajectory", 2, true);
        // 根据参数决定是否发布地图。
        if (options.publish_map) {
            // 创建地图消息 header。
            std_msgs::Header header;
            // 静态地图使用当前时间戳。
            header.stamp = ros::Time::now();
            // 地图坐标系。
            header.frame_id = options.frame_id;
            // 从体素哈希地图取点。
            const auto map_points = global_map.Pointcloud();
            // 转成 PointCloud2 并发布。
            map_publisher.publish(*genz_icp_ros::utils::EigenToPointCloud2(map_points, header));
        }
    }

    // 打开 CSV 文件。
    std::ofstream csv(options.output_csv);
    // CSV 打开失败时退出。
    if (!csv) {
        // 打印失败路径。
        std::cerr << "Failed to open output CSV: " << options.output_csv << std::endl;
        // 返回错误码 4。
        return 4;
    }
    // 写入 CSV 表头。
    csv << "stamp,elapsed,frame,scan_points,source_points,"
           "icp_x,icp_y,icp_z,icp_yaw,ins_x,ins_y,ins_z,ins_yaw,"
           "error_xy,error_z,error_yaw_rad\n";

    // 保存打开的 bag 对象；必须保持生命周期，否则 rosbag::View 会失效。
    std::vector<std::unique_ptr<rosbag::Bag>> bags;
    // rosbag::View 会按时间顺序遍历多个 bag 的消息。
    rosbag::View view;
    // 遍历每个 bag 文件。
    for (const auto &path : bag_paths) {
        // 创建 bag 对象。
        auto bag = std::make_unique<rosbag::Bag>();
        // 以只读方式打开 bag。
        bag->open(path, rosbag::bagmode::Read);
        // 只查询点云和 INS 两个话题，避免读取无关消息。
        view.addQuery(*bag, rosbag::TopicQuery({options.cloud_topic, options.ins_topic}));
        // 保存 bag 对象生命周期。
        bags.emplace_back(std::move(bag));
    }

    // 记录整个 view 的起始时间，用于 elapsed。
    const ros::Time begin_time = view.getBeginTime();
    // 最新 INS 位姿缓存。
    std::optional<Sophus::SE3d> latest_ins;
    // 上一帧 INS 位姿缓存。
    std::optional<Sophus::SE3d> previous_ins;
    // 上一帧 ICP 位姿缓存。
    std::optional<Sophus::SE3d> previous_icp;
    // 已处理点云帧号。
    size_t frame_index = 0;
    // 累计 xy 误差，用于最后算平均值。
    double sum_xy_error = 0.0;
    // 最大 xy 误差。
    double max_xy_error = 0.0;

    // 按时间顺序遍历 bag 中 INS 和点云消息。
    //
    // 循环里有两类消息：
    //   1. INS：只更新 latest_ins，作为后续点云帧的参考位姿。
    //   2. PointCloud2：如果 latest_ins 已经存在，就执行一次 scan-to-map 配准。
    //
    // 因为 rosbag::View 已经按时间排序，所以点云使用的是“该点云之前最近一次收到的 INS”。
    for (const auto &message : view) {
        // 计算当前消息相对整个 bag 起点的时间。
        const double elapsed = (message.getTime() - begin_time).toSec();
        // 如果设置了运行时长限制，超过后退出循环。
        if (options.duration > 0.0 && elapsed > options.duration) break;

        // 如果当前消息是 INS，就更新 latest_ins。
        if (message.getTopic() == options.ins_topic || ("/" + message.getTopic()) == options.ins_topic) {
            // 反序列化成 nav_msgs/Odometry。
            const auto odom = message.instantiate<nav_msgs::Odometry>();
            // 成功反序列化后转换为 Sophus 位姿。
            if (odom) latest_ins = OdomToSophus(*odom);
            // INS 消息处理完毕，继续下一条消息。
            continue;
        }

        // 如果不是目标点云话题，就跳过。
        if (!(message.getTopic() == options.cloud_topic ||
              ("/" + message.getTopic()) == options.cloud_topic)) {
            // 继续下一条消息。
            continue;
        }
        // 反序列化成 PointCloud2。
        const auto cloud_msg = message.instantiate<sensor_msgs::PointCloud2>();
        // 没有点云或还没有 INS 初值时跳过。
        if (!cloud_msg || !latest_ins) continue;

        // ROS 点云转 Eigen 点数组。
        const auto raw_points = genz_icp_ros::utils::PointCloud2ToEigen(cloud_msg);
        // 距离裁剪。
        const auto cropped = genz_icp::Preprocess(raw_points, options.max_range, options.min_range);
        // 当前帧下采样。
        const auto source = genz_icp::VoxelDownsample(cropped, options.scan_voxel_size);

        // 第一帧默认使用当前 INS 作为 ICP 初值。
        Sophus::SE3d initial_guess = *latest_ins;
        // 如果已有上一帧 ICP，则用上一帧结果预测当前位姿。
        if (previous_icp) {
            // 默认只用上一帧 ICP。
            initial_guess = *previous_icp;
            // 如果启用 INS prediction，就叠加 INS 帧间增量。
            if (options.use_ins_prediction && previous_ins) {
                // 计算 INS 帧间运动。
                const Sophus::SE3d delta_ins = previous_ins->inverse() * (*latest_ins);
                // 叠加到上一帧 ICP 位姿上。
                initial_guess = (*previous_icp) * delta_ins;
            }
        }

        // 做 scan-to-map 配准。
        // 离线模式这里的逻辑和在线 CloudCallback 中基本一致。
        const auto [pose, planar_points, non_planar_points] =
            registration.RegisterFrame(source,
                                       global_map,
                                       initial_guess,
                                       options.max_correspondence_distance,
                                       options.kernel);

        // 离线模式如果开启发布，就把结果发给 RViz。
        if (options.publish && ros::ok()) {
            // 创建输出 header。
            std_msgs::Header header;
            // 离线发布时使用当前 ROS 时间，模拟实时播放。
            header.stamp = ros::Time::now();
            // 输出坐标系。
            header.frame_id = options.frame_id;

            // 创建 Odometry 消息。
            nav_msgs::Odometry odom_msg;
            // 写入 header。
            odom_msg.header = header;
            // 子坐标系写 base_link。
            odom_msg.child_frame_id = "base_link";
            // 写入 ICP 位姿。
            odom_msg.pose.pose = SophusToPose(pose);
            // 发布 odometry。
            odom_publisher.publish(odom_msg);

            // 创建 Path 中当前帧的 PoseStamped。
            geometry_msgs::PoseStamped pose_msg;
            // 写入 header。
            pose_msg.header = header;
            // 写入 pose。
            pose_msg.pose = odom_msg.pose.pose;
            // 更新 Path 时间戳。
            path_msg.header.stamp = header.stamp;
            // 把当前位姿追加到轨迹。
            path_msg.poses.push_back(pose_msg);
            // 发布轨迹。
            path_publisher.publish(path_msg);

            // 发布配准后点云，观察 scan 是否贴合地图。
            aligned_scan_publisher.publish(
                *genz_icp_ros::utils::EigenToPointCloud2(source, pose, header));
            // 发布参与本次 ICP 优化的平面点，观察点到面约束分布。
            planar_points_publisher.publish(
                *genz_icp_ros::utils::EigenToPointCloud2(planar_points, header));
            // 发布参与本次 ICP 优化的非平面点，观察点到点约束分布。
            non_planar_points_publisher.publish(
                *genz_icp_ros::utils::EigenToPointCloud2(non_planar_points, header));
            // 处理一次 ROS 回调，保证发布队列刷新。
            ros::spinOnce();

            // 人为 sleep，避免离线模式刷得太快 RViz 来不及显示。
            if (options.replay_rate > 0.0) {
                // replay_rate 越大，sleep 越短。
                ros::Duration(0.02 / options.replay_rate).sleep();
            }
        }

        // 计算 ICP 位姿和 INS 位姿的平移误差。
        // 如果 INS 和地图坐标系一致，这可以近似作为定位误差；
        // 如果二者存在坐标系偏移，则需要先做坐标对齐再评价。
        const Eigen::Vector3d err = pose.translation() - latest_ins->translation();
        // 计算 xy 平面误差。
        const double xy_error = err.head<2>().norm();
        // 计算 yaw 误差。
        const double yaw_error = WrapAngle(YawFromPose(pose) - YawFromPose(*latest_ins));
        // 累加 xy 误差。
        sum_xy_error += xy_error;
        // 更新最大 xy 误差。
        max_xy_error = std::max(max_xy_error, xy_error);

        // 写一行 CSV。
        csv << std::fixed << std::setprecision(9)
            << cloud_msg->header.stamp.toSec() << "," << elapsed << "," << frame_index << ","
            << raw_points.size() << "," << source.size() << ","
            << pose.translation().x() << "," << pose.translation().y() << ","
            << pose.translation().z() << "," << YawFromPose(pose) << ","
            << latest_ins->translation().x() << "," << latest_ins->translation().y() << ","
            << latest_ins->translation().z() << "," << YawFromPose(*latest_ins) << ","
            << xy_error << "," << err.z() << "," << yaw_error << "\n";

        // 控制台打印当前帧摘要。
        std::cout << std::fixed << std::setprecision(3)
                  << "frame " << frame_index
                  << " t=" << elapsed
                  << " source=" << source.size()
                  << " corr=" << (planar_points.size() + non_planar_points.size())
                  << " xy_err=" << xy_error
                  << " yaw_err=" << yaw_error << std::endl;

        // 保存当前 ICP 位姿，供下一帧预测。
        previous_icp = pose;
        // 保存当前 INS 位姿，供下一帧 INS prediction。
        previous_ins = latest_ins;
        // 帧号加一。
        ++frame_index;
    }

    // 如果没有处理任何点云，说明 topic 或 bag 有问题。
    if (frame_index == 0) {
        // 打印点云话题名，方便排查 remap。
        std::cerr << "No point cloud frames processed. Check topic: " << options.cloud_topic
                  << std::endl;
        // 返回错误码 5。
        return 5;
    }

    // 打印离线运行总结。
    std::cout << "Processed frames: " << frame_index
              << ", mean XY error vs INS: " << (sum_xy_error / static_cast<double>(frame_index))
              << ", max XY error: " << max_xy_error
              << "\nCSV: " << options.output_csv << std::endl;
    // 正常结束。
    return 0;
}
