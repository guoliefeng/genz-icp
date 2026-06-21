// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

#include <deque>
#include <optional>
#include <string>
#include <vector>

// ROS
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace genz_icp_ros {

class LocLIO {
public:
    /// ROS1 LIO 节点构造函数：读取参数、初始化 GenZ-ICP、建立订阅和发布。
    LocLIO(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
    struct ImuSample {
        ros::Time stamp;
        Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
    };

    struct ImuPrediction {
        Sophus::SE3d pose;
        Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
        size_t sample_count{0};
    };

    /// INS 回调：首帧 INS 作为 LO 初始位姿，后续只缓存参考位姿。
    void InsCallback(const nav_msgs::Odometry::ConstPtr &msg);

    /// IMU 回调：缓存角速度和线加速度，IMU 到 base_link 外参按单位矩阵处理。
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    /// 从上一帧 LiDAR 校正状态积分到当前点云时刻，生成 ICP 绝对初值。
    std::optional<ImuPrediction> PredictPoseWithImu(const ros::Time &target_stamp) const;

    /// 用首个 LiDAR/ICP pose 初始化 IMU 状态。
    void ResetImuState(const Sophus::SE3d &pose, const ros::Time &stamp);

    /// 用 ICP 结果校正 IMU pose，并用相邻 ICP 位移更新世界系速度。
    void CorrectImuState(const Sophus::SE3d &corrected_pose,
                         const ros::Time &stamp,
                         const std::optional<ImuPrediction> &prediction);

    /// 发布 IMU 给出的本帧 ICP 初值，方便 RViz/rosbag 调试预测质量。
    void PublishImuPrediction(const ImuPrediction &prediction,
                              const ros::Time &stamp,
                              const std::string &child_frame_id);

    /// 点云回调：每收到一帧 PointCloud2，就调用 GenZ-ICP 做一次配准。
    void RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg);

    /// 发布当前估计位姿：包括 Odometry、Path，以及可选 TF。
    void PublishOdometry(const Sophus::SE3d &pose,
                         const ros::Time &stamp,
                         const std::string &cloud_frame_id);

    /// 发布调试点云：局部地图、平面点、非平面点，主要给 RViz 观察算法状态。
    void PublishClouds(const ros::Time &stamp,
                       const std::string &cloud_frame_id,
                       const std::vector<Eigen::Vector3d> &planar_points,
                       const std::vector<Eigen::Vector3d> &non_planar_points);

    /// 发布 INS 参考轨迹，方便 RViz 和 LO 轨迹叠加对比。
    void PublishInsTrajectory(const nav_msgs::Odometry &odom);

    /// 从 TF 树中查询两个坐标系之间的变换，并转换成 Sophus::SE3d。
    Sophus::SE3d LookupTransform(const std::string &target_frame,
                                 const std::string &source_frame) const;

    /// 读取 PCD 全局地图，仅用于 RViz 显示，不参与 LO 匹配。
    void LoadAndPublishGlobalMap();

    /// 确保全局 PCD 地图已加载；精配准初始化和 RViz 显示共用同一份地图。
    bool LoadGlobalMapIfNeeded(const std::optional<Eigen::Vector3d> &crop_center = std::nullopt);

    /// 定时重发全局地图，避免 RViz 晚启动或显示临时丢失。
    void PublishGlobalMap(const ros::TimerEvent &event);

    /// 使用 INS/NDT + GenZ-ICP 对首帧点云做初始化配准。
    bool InitializeFromMatchedPose(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                   const std::vector<Eigen::Vector3d> &points);

    /// 用 GenZ-ICP 在全局地图上做精配准，并返回匹配比例作为质量分数。
    std::tuple<Sophus::SE3d, double, size_t, size_t> FineAlignInitialPose(
        const std::vector<Eigen::Vector3d> &source,
        const Sophus::SE3d &initial_guess) const;

    /// 用 NDT 在全局地图上做粗配准。
    std::optional<Sophus::SE3d> CoarseAlignWithNdt(const std::vector<Eigen::Vector3d> &source,
                                                   const Sophus::SE3d &initial_guess) const;

    /// 把 ROS Odometry 转成 Sophus SE3。
    Sophus::SE3d OdomToSophus(const nav_msgs::Odometry &odom) const;

    /// 把 Sophus SE3 转成 PCL/NDT 使用的 4x4 float 矩阵。
    Eigen::Matrix4f SophusToMatrix4f(const Sophus::SE3d &pose) const;

    /// 把 PCL/NDT 输出的 4x4 float 矩阵转成 Sophus SE3。
    Sophus::SE3d Matrix4fToSophus(const Eigen::Matrix4f &matrix) const;

    /// 公共命名空间 NodeHandle，用于订阅普通话题。
    ros::NodeHandle nh_;
    /// 私有命名空间 NodeHandle，用于读取私有参数和发布节点私有输出。
    ros::NodeHandle pnh_;
    /// ROS 订阅/发布队列长度；这里为 1，表示优先处理最新点云。
    int queue_size_{1};

    /// TF 广播器：当 publish_odom_tf=true 时发布 odom->base 的 TF。
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    /// TF 缓冲区：缓存坐标系变换，供 LookupTransform 查询。
    tf2_ros::Buffer tf2_buffer_;
    /// TF 监听器：持续把 ROS TF 写入 tf2_buffer_。
    tf2_ros::TransformListener tf2_listener_;
    /// 是否发布里程计 TF。
    bool publish_odom_tf_{false};
    /// 是否发布调试点云。
    bool publish_debug_clouds_{true};
    /// 是否用 IMU 预积分替换 GenZ-ICP 内部匀速模型。
    bool use_imu_prediction_{true};
    /// IMU 不可用时是否回退到原匀速模型。
    bool imu_fallback_to_constant_velocity_{true};
    /// IMU 话题。
    std::string imu_topic_{"/ins_driver/imu"};
    /// 当前数据的加速度约为 1g，默认转换为 m/s^2。
    double imu_accel_scale_{9.80665};
    /// 世界系重力大小，假定世界 z 轴向上。
    double imu_gravity_magnitude_{9.80665};
    /// 单个 IMU 积分步允许的最大时间间隔。
    double imu_max_sample_gap_{0.05};
    /// LiDAR 两帧之间允许的最大 IMU 预测时长。
    double imu_max_prediction_interval_{0.25};
    /// ICP 速度观测对 IMU 速度状态的校正权重，1 表示完全采用 ICP 速度。
    double imu_velocity_correction_gain_{1.0};
    /// 防止异常 IMU 消息产生不合理预测。
    double imu_max_acceleration_{30.0};
    double imu_max_angular_velocity_{3.0};
    /// 是否等待首帧 INS 作为初始位姿。
    bool use_ins_init_{true};
    /// 是否已经用 INS 初始化 GenZ-ICP。
    bool initialized_from_ins_{false};
    /// 是否用全局地图精配准修正首帧 INS 初值。
    bool refine_ins_init_{true};
    /// 精配准分数阈值；分数为有效对应点数/source 点数，越高越可信。
    double init_fine_score_threshold_{0.35};
    /// 精配准最少对应点数。
    int init_min_correspondences_{300};
    /// 初始化精配准当前帧下采样体素。
    double init_scan_voxel_size_{0.6};
    /// 初始化精配准最大对应距离。
    double init_max_correspondence_distance_{2.0};
    /// 初始化精配准鲁棒核。
    double init_kernel_{0.7};
    /// 初始化全局地图裁剪半径；>0 时以首帧 INS xy 为中心裁剪，<=0 时使用整张下采样地图。
    double init_map_crop_radius_{160.0};
    /// NDT 粗配准开关；精配准分数低时才触发。
    bool init_use_ndt_fallback_{true};
    /// NDT source 下采样体素。
    double init_ndt_source_voxel_size_{1.0};
    /// NDT map 下采样体素。
    double init_ndt_map_voxel_size_{1.0};
    /// NDT 分辨率。
    double init_ndt_resolution_{2.0};
    /// NDT 最大迭代次数。
    int init_ndt_max_iterations_{35};
    /// NDT transformation epsilon。
    double init_ndt_transformation_epsilon_{0.01};
    /// NDT step size。
    double init_ndt_step_size_{0.1};
    /// NDT 最大可接受 fitness score；越小越好。
    double init_ndt_max_fitness_score_{3.0};
    /// 是否发布全局 PCD 地图用于 RViz 参考。
    bool publish_global_map_{false};
    /// 全局地图路径；只用于显示。
    std::string map_path_{"/home/guoli/data/yangpu/loc/map.pcd"};
    /// INS 话题名。
    std::string ins_topic_{"/localization/ins"};
    /// 全局地图发布坐标系。
    std::string global_map_frame_{"map"};
    /// 全局地图重发周期；<=0 时只依赖 latched publisher。
    double global_map_publish_period_{2.0};

    /// 点云订阅器，订阅 launch 中 remap 到 pointcloud_topic 的 LiDAR 点云。
    ros::Subscriber pointcloud_sub_;
    /// INS 订阅器。
    ros::Subscriber ins_sub_;
    /// IMU 订阅器。
    ros::Subscriber imu_sub_;

    /// 里程计发布器，输出 /genz/odometry。
    ros::Publisher odom_publisher_;
    /// 局部地图发布器，输出 /genz/local_map。
    ros::Publisher map_publisher_;
    /// 轨迹发布器，输出 /genz/trajectory。
    ros::Publisher traj_publisher_;
    /// INS 参考轨迹发布器，输出 /genz/ins_trajectory。
    ros::Publisher ins_traj_publisher_;
    /// IMU 预积分预测位姿发布器。
    ros::Publisher imu_prediction_publisher_;
    /// 平面点发布器，输出 /genz/planar_points。
    ros::Publisher planar_points_publisher_;
    /// 非平面点发布器，输出 /genz/non_planar_points。
    ros::Publisher non_planar_points_publisher_;
    /// 全局 PCD 地图发布器，输出 /genz/global_map。
    ros::Publisher global_map_publisher_;
    /// 全局地图定时重发器。
    ros::Timer global_map_timer_;
    /// Path 消息缓存；每一帧都会追加一个 PoseStamped。
    nav_msgs::Path path_msg_;
    /// INS Path 消息缓存。
    nav_msgs::Path ins_path_msg_;
    /// 缓存后的全局地图消息。
    std::optional<sensor_msgs::PointCloud2> global_map_msg_;
    /// 全局地图 Eigen 点；仅在 RViz 发布全局地图时缓存，避免初始化调试时多占一份大地图内存。
    std::vector<Eigen::Vector3d> global_map_points_;
    /// 全局地图体素哈希，用于 GenZ-ICP 精配准初始化。
    std::optional<genz_icp::VoxelHashMap> global_map_voxel_;
    /// NDT 使用的下采样全局地图。
    pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_target_map_{new pcl::PointCloud<pcl::PointXYZ>};
    /// 全局地图是否已经成功加载。
    bool global_map_loaded_{false};
    /// 当前初始化地图的裁剪中心；用于日志和避免重复加载。
    std::optional<Eigen::Vector3d> global_map_crop_center_;
    /// 最近收到的 INS 位姿；精配准初始化会用它作为第一候选初值。
    std::optional<Sophus::SE3d> latest_ins_pose_;

    /// 按时间排列的 IMU 消息缓冲。
    std::deque<ImuSample> imu_buffer_;
    /// 上一帧 ICP 校正后的位姿和时间，是下一次 IMU 预积分的起点。
    std::optional<Sophus::SE3d> last_corrected_pose_;
    std::optional<ros::Time> last_lidar_stamp_;
    /// 世界坐标系速度状态，每帧由 ICP 位移校正。
    Eigen::Vector3d imu_world_velocity_{Eigen::Vector3d::Zero()};
    /// IMU 预测成功/回退计数，用于运行诊断。
    size_t imu_prediction_success_count_{0};
    size_t imu_prediction_fallback_count_{0};

    /// GenZ-ICP 原始里程计流水线对象。
    genz_icp::pipeline::GenZICP odometry_;
    /// GenZ-ICP 参数配置。
    genz_icp::pipeline::GenZConfig config_;

    /// 输出里程计所在坐标系，默认 odom。
    std::string odom_frame_{"odom"};
    /// 车体坐标系；为空时直接使用点云 frame。
    std::string base_frame_{};
};

}  // namespace genz_icp_ros
