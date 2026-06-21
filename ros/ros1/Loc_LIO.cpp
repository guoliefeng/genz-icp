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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

// GenZ-ICP-ROS
#include "Loc_LIO.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 1 headers
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace genz_icp_ros {

// 把 Utils.hpp 里的点云转换函数引入当前命名空间，代码更短。
using utils::EigenToPointCloud2;
// 获取 PointCloud2 中每个点的相对时间戳，用于运动畸变补偿。
using utils::GetTimestamps;
// 把 ROS PointCloud2 转成 GenZ-ICP 使用的 Eigen::Vector3d 点数组。
using utils::PointCloud2ToEigen;

LocLIO::LocLIO(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf2_listener_(tf2_ros::TransformListener(tf2_buffer_)) {
    // 读取车体坐标系名称；为空时表示直接使用点云坐标系。
    pnh_.param("base_frame", base_frame_, base_frame_);
    // 读取里程计输出坐标系名称，默认 odom。
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
    // 读取是否发布 odom->base 的 TF。
    pnh_.param("publish_odom_tf", publish_odom_tf_, false);
    // 读取是否发布调试点云，RViz 调试时使用。
    pnh_.param("visualize", publish_debug_clouds_, publish_debug_clouds_);
    // 是否用 IMU 预积分替换 GenZ-ICP 内部匀速预测。
    pnh_.param("use_imu_prediction", use_imu_prediction_, use_imu_prediction_);
    pnh_.param("imu_fallback_to_constant_velocity", imu_fallback_to_constant_velocity_,
               imu_fallback_to_constant_velocity_);
    pnh_.param("imu_topic", imu_topic_, imu_topic_);
    pnh_.param("imu_accel_scale", imu_accel_scale_, imu_accel_scale_);
    pnh_.param("imu_gravity_magnitude", imu_gravity_magnitude_, imu_gravity_magnitude_);
    pnh_.param("imu_max_sample_gap", imu_max_sample_gap_, imu_max_sample_gap_);
    pnh_.param("imu_max_prediction_interval", imu_max_prediction_interval_,
               imu_max_prediction_interval_);
    pnh_.param("imu_velocity_correction_gain", imu_velocity_correction_gain_,
               imu_velocity_correction_gain_);
    pnh_.param("imu_max_acceleration", imu_max_acceleration_, imu_max_acceleration_);
    pnh_.param("imu_max_angular_velocity", imu_max_angular_velocity_, imu_max_angular_velocity_);
    imu_velocity_correction_gain_ = std::clamp(imu_velocity_correction_gain_, 0.0, 1.0);
    // 是否等待首帧 INS，并把 INS pose 作为 LO 初始位姿。
    pnh_.param("use_ins_init", use_ins_init_, use_ins_init_);
    // 是否把首帧 INS 先放进全局 PCD 地图做一次初始化配准，再用配准 pose 初始化 LO。
    pnh_.param("refine_ins_init", refine_ins_init_, refine_ins_init_);
    // 初始化精配准分数阈值，分数是有效对应点数量/source 点数量。
    pnh_.param("init_fine_score_threshold", init_fine_score_threshold_, init_fine_score_threshold_);
    // 初始化精配准最少对应点数。
    pnh_.param("init_min_correspondences", init_min_correspondences_, init_min_correspondences_);
    // 初始化精配准当前帧下采样体素。
    pnh_.param("init_scan_voxel", init_scan_voxel_size_, init_scan_voxel_size_);
    // 初始化精配准最大对应距离。
    pnh_.param("init_max_corr", init_max_correspondence_distance_, init_max_correspondence_distance_);
    // 初始化精配准鲁棒核。
    pnh_.param("init_kernel", init_kernel_, init_kernel_);
    // 初始化地图裁剪半径；参考 Loc_Map 的 map_radius，以首帧 INS 的 xy 为中心裁剪。
    pnh_.param("init_map_crop_radius", init_map_crop_radius_, init_map_crop_radius_);
    // 精配准分数低时是否启用 NDT 粗配准回退。
    pnh_.param("init_use_ndt_fallback", init_use_ndt_fallback_, init_use_ndt_fallback_);
    // NDT source/map 下采样和优化参数。
    pnh_.param("init_ndt_source_voxel", init_ndt_source_voxel_size_, init_ndt_source_voxel_size_);
    pnh_.param("init_ndt_map_voxel", init_ndt_map_voxel_size_, init_ndt_map_voxel_size_);
    pnh_.param("init_ndt_resolution", init_ndt_resolution_, init_ndt_resolution_);
    pnh_.param("init_ndt_max_iterations", init_ndt_max_iterations_, init_ndt_max_iterations_);
    pnh_.param("init_ndt_transformation_epsilon", init_ndt_transformation_epsilon_, init_ndt_transformation_epsilon_);
    pnh_.param("init_ndt_step_size", init_ndt_step_size_, init_ndt_step_size_);
    pnh_.param("init_ndt_max_fitness_score", init_ndt_max_fitness_score_, init_ndt_max_fitness_score_);
    // INS 输入话题。
    pnh_.param("ins_topic", ins_topic_, ins_topic_);
    // 是否加载并发布全局 PCD 地图；这个地图只用于 RViz 参考，不参与 LO 匹配。
    pnh_.param("publish_global_map", publish_global_map_, publish_global_map_);
    // 全局地图路径。
    pnh_.param("map_path", map_path_, map_path_);
    // 全局地图坐标系，通常应和 INS pose 所在坐标系一致。
    pnh_.param("global_map_frame", global_map_frame_, global_map_frame_);
    // 全局地图重发周期。publisher 本身是 latched，这里定时重发是为了照顾 RViz/仿真时间显示。
    pnh_.param("global_map_publish_period", global_map_publish_period_, global_map_publish_period_);
    // 读取最大有效点云距离，超过该距离的点会被裁掉。
    pnh_.param("max_range", config_.max_range, config_.max_range);
    // 读取最小有效点云距离，过近点会被裁掉。
    pnh_.param("min_range", config_.min_range, config_.min_range);
    // 读取是否启用 deskew，启用后需要点云中有时间戳字段。
    pnh_.param("deskew", config_.deskew, config_.deskew);
    // 读取体素下采样尺寸，影响速度和精度。
    pnh_.param("voxel_size", config_.voxel_size, config_.max_range / 100.0);
    // 读取局部地图清理半径，超出该半径的地图点会被移除。
    pnh_.param("map_cleanup_radius", config_.map_cleanup_radius, config_.max_range);
    // 读取平面判断阈值，用于区分平面点和非平面点。
    pnh_.param("planarity_threshold", config_.planarity_threshold, config_.planarity_threshold);
    // 读取每个体素最多保存多少个点。
    pnh_.param("max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    // 读取希望每帧参与配准的大致点数，算法会自适应体素尺寸。
    pnh_.param("desired_num_voxelized_points", config_.desired_num_voxelized_points, config_.desired_num_voxelized_points);
    // 读取 ICP 初始对应距离阈值。
    pnh_.param("initial_threshold", config_.initial_threshold, config_.initial_threshold);
    // 读取最小运动阈值，用于自适应阈值模型。
    pnh_.param("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    // 读取 ICP 最大迭代次数。
    pnh_.param("max_num_iterations", config_.max_num_iterations, config_.max_num_iterations);
    // 读取 ICP 收敛判据。
    pnh_.param("convergence_criterion", config_.convergence_criterion, config_.convergence_criterion);
    // 防御性检查：最大距离小于最小距离时，参数明显不合理。
    if (config_.max_range < config_.min_range) {
        // 给出 ROS 警告，提醒用户参数配置有问题。
        ROS_WARN("[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
        // 自动把 min_range 置零，避免所有点都被裁掉。  
        config_.min_range = 0.0;
    }

    // 用读取到的配置构造 GenZ-ICP 主流水线对象。
    odometry_ = genz_icp::pipeline::GenZICP(config_);
    // 日常运行和资源测试时关闭底层终端动画输出，避免 roslaunch 控制台刷屏。
    odometry_.SetTerminalStatusEnabled(false);

    // 订阅点云话题；launch 会把 pointcloud_topic remap 成真实 LiDAR topic。
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud_topic", queue_size_,
                                                              &LocLIO::RegisterFrame, this);
    // 订阅 INS；首帧 INS 会用于设置 GenZ-ICP 初始 pose。
    ins_sub_ = nh_.subscribe<nav_msgs::Odometry>(ins_topic_, 200, &LocLIO::InsCallback, this);
    // IMU 与点云外参按单位矩阵处理；回调只缓存，点云回调按时间戳执行预积分。
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic_, 1000, &LocLIO::ImuCallback, this);

    // 发布 GenZ-ICP 估计的里程计。
    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("/genz/odometry", queue_size_);
    // 发布累计轨迹 Path。
    traj_publisher_ = pnh_.advertise<nav_msgs::Path>("/genz/trajectory", queue_size_);
    // 发布 INS 参考轨迹 Path，方便 RViz 和 /genz/trajectory 叠加比较。
    ins_traj_publisher_ = pnh_.advertise<nav_msgs::Path>("/genz/ins_trajectory", 2, true);
    imu_prediction_publisher_ =
        pnh_.advertise<nav_msgs::Odometry>("/genz/imu_prediction", queue_size_);
    // 只有 visualize=true 时才创建调试点云发布器，避免不必要的开销。
    if (publish_debug_clouds_) {
        // 发布 GenZ-ICP 在线维护的局部地图。
        map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/local_map", queue_size_);
        // 发布当前帧中用于点到面约束的平面点。
        planar_points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/planar_points", queue_size_);
        // 发布当前帧中用于点到点约束的非平面点。
        non_planar_points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/non_planar_points", queue_size_);
    }
    // 全局地图使用 latched publisher：RViz 晚启动也能收到最后一次地图消息。
    if (publish_global_map_ || refine_ins_init_) {
        global_map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/global_map", 1, true);
        if (!refine_ins_init_ && !LoadGlobalMapIfNeeded()) {
            ROS_ERROR("Failed to load global map for RViz publishing.");
        }
        if (publish_global_map_ && !refine_ins_init_) {
            LoadAndPublishGlobalMap();
        }
        if (publish_global_map_ && global_map_publish_period_ > 0.0) {
            global_map_timer_ = nh_.createTimer(ros::Duration(global_map_publish_period_),
                                                &LocLIO::PublishGlobalMap,
                                                this);
        }
    }
    // TF buffer 使用独立线程，避免查询 TF 时阻塞主点云回调。
    tf2_buffer_.setUsingDedicatedThread(true);
    // Path 的坐标系固定为 odom_frame_。
    path_msg_.header.frame_id = odom_frame_;
    // INS Path 默认也放在 odom_frame_ 下；loc_lo.launch 默认 odom_frame:=map。
    ins_path_msg_.header.frame_id = odom_frame_;

    if (use_ins_init_) {
        if (refine_ins_init_) {
            ROS_INFO_STREAM("LocLIO is waiting for INS and first cloud to refine init pose on map: " << map_path_);
        } else {
            ROS_INFO_STREAM("LocLIO is waiting for INS init pose on " << ins_topic_);
        }
    } else {
        initialized_from_ins_ = true;
        ROS_WARN("LocLIO use_ins_init=false, odometry will start from identity pose");
    }

    // 节点初始化完成。
    ROS_INFO_STREAM("GenZ-ICP ROS 1 LocLIO Node Initialized. IMU prediction="
                    << use_imu_prediction_ << ", imu_topic=" << imu_topic_
                    << ", accel_scale=" << imu_accel_scale_
                    << ", IMU->base extrinsic=identity");
}

Sophus::SE3d LocLIO::OdomToSophus(const nav_msgs::Odometry &odom) const {
    const auto &p = odom.pose.pose.position;
    const auto &q = odom.pose.pose.orientation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    quat.normalize();
    return Sophus::SE3d(quat, Eigen::Vector3d(p.x, p.y, p.z));
}

void LocLIO::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    const Eigen::Vector3d angular_velocity(msg->angular_velocity.x,
                                           msg->angular_velocity.y,
                                           msg->angular_velocity.z);
    const Eigen::Vector3d linear_acceleration(msg->linear_acceleration.x,
                                              msg->linear_acceleration.y,
                                              msg->linear_acceleration.z);
    if (!angular_velocity.allFinite() || !linear_acceleration.allFinite()) return;

    // 外参为单位矩阵，因此 IMU 测量无需旋转即可作为 base_link 体坐标系测量。
    imu_buffer_.push_back({msg->header.stamp, angular_velocity,
                           linear_acceleration * imu_accel_scale_});
    // 只保留最近 5 秒，避免未播放点云或点云中断时缓冲无限增长。
    const ros::Time keep_after = msg->header.stamp - ros::Duration(5.0);
    while (imu_buffer_.size() > 2 && imu_buffer_[1].stamp < keep_after) {
        imu_buffer_.pop_front();
    }
}

std::optional<LocLIO::ImuPrediction> LocLIO::PredictPoseWithImu(
    const ros::Time &target_stamp) const {
    if (!use_imu_prediction_ || !last_corrected_pose_ || !last_lidar_stamp_ ||
        imu_buffer_.empty()) {
        return std::nullopt;
    }

    const double prediction_interval = (target_stamp - *last_lidar_stamp_).toSec();
    if (prediction_interval <= 0.0 || prediction_interval > imu_max_prediction_interval_) {
        return std::nullopt;
    }

    const ImuSample *measurement = nullptr;
    for (const auto &sample : imu_buffer_) {
        if (sample.stamp <= *last_lidar_stamp_) {
            measurement = &sample;
        } else {
            break;
        }
    }
    if (!measurement) {
        measurement = &imu_buffer_.front();
        if ((measurement->stamp - *last_lidar_stamp_).toSec() > imu_max_sample_gap_) {
            return std::nullopt;
        }
    }

    Sophus::SO3d rotation = last_corrected_pose_->so3();
    Eigen::Vector3d position = last_corrected_pose_->translation();
    Eigen::Vector3d velocity = imu_world_velocity_;
    ros::Time integration_stamp = *last_lidar_stamp_;
    size_t sample_count = 0;

    auto integrate = [&](const Eigen::Vector3d &angular_velocity,
                         const Eigen::Vector3d &linear_acceleration,
                         double dt) -> bool {
        if (dt <= 0.0) return true;
        if (dt > imu_max_sample_gap_ ||
            angular_velocity.norm() > imu_max_angular_velocity_ ||
            linear_acceleration.norm() > imu_max_acceleration_) {
            return false;
        }
        // 中点姿态用于把比力旋转到世界系；世界 z 向上，所以重力沿 -z。
        const Sophus::SO3d half_rotation =
            rotation * Sophus::SO3d::exp(0.5 * angular_velocity * dt);
        const Eigen::Vector3d world_acceleration =
            half_rotation * linear_acceleration +
            Eigen::Vector3d(0.0, 0.0, -imu_gravity_magnitude_);
        position += velocity * dt + 0.5 * world_acceleration * dt * dt;
        velocity += world_acceleration * dt;
        rotation = rotation * Sophus::SO3d::exp(angular_velocity * dt);
        return position.allFinite() && velocity.allFinite();
    };

    for (const auto &sample : imu_buffer_) {
        if (sample.stamp <= *last_lidar_stamp_) continue;
        if (sample.stamp > target_stamp) break;
        const double dt = (sample.stamp - integration_stamp).toSec();
        const Eigen::Vector3d angular_velocity =
            0.5 * (measurement->angular_velocity + sample.angular_velocity);
        const Eigen::Vector3d linear_acceleration =
            0.5 * (measurement->linear_acceleration + sample.linear_acceleration);
        if (!integrate(angular_velocity, linear_acceleration, dt)) return std::nullopt;
        measurement = &sample;
        integration_stamp = sample.stamp;
        ++sample_count;
    }

    const double final_dt = (target_stamp - integration_stamp).toSec();
    if (!integrate(measurement->angular_velocity, measurement->linear_acceleration, final_dt)) {
        return std::nullopt;
    }
    return ImuPrediction{Sophus::SE3d(rotation, position), velocity, sample_count};
}

void LocLIO::ResetImuState(const Sophus::SE3d &pose, const ros::Time &stamp) {
    last_corrected_pose_ = pose;
    last_lidar_stamp_ = stamp;
    imu_world_velocity_.setZero();
}

void LocLIO::CorrectImuState(const Sophus::SE3d &corrected_pose,
                             const ros::Time &stamp,
                             const std::optional<ImuPrediction> &prediction) {
    if (!last_corrected_pose_ || !last_lidar_stamp_) {
        ResetImuState(corrected_pose, stamp);
        return;
    }
    const double dt = (stamp - *last_lidar_stamp_).toSec();
    if (dt > 0.0 && dt <= imu_max_prediction_interval_) {
        const Eigen::Vector3d icp_velocity =
            (corrected_pose.translation() - last_corrected_pose_->translation()) / dt;
        const Eigen::Vector3d prior_velocity = prediction ? prediction->velocity : imu_world_velocity_;
        imu_world_velocity_ =
            (1.0 - imu_velocity_correction_gain_) * prior_velocity +
            imu_velocity_correction_gain_ * icp_velocity;
    } else {
        imu_world_velocity_.setZero();
    }
    last_corrected_pose_ = corrected_pose;
    last_lidar_stamp_ = stamp;

    // 为下一帧保留起始时刻之前最近的一条 IMU，其余旧数据可以删除。
    while (imu_buffer_.size() > 2 && imu_buffer_[1].stamp <= stamp) {
        imu_buffer_.pop_front();
    }
}

void LocLIO::PublishImuPrediction(const ImuPrediction &prediction,
                                  const ros::Time &stamp,
                                  const std::string &child_frame_id) {
    nav_msgs::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = child_frame_id;
    msg.pose.pose = tf2::sophusToPose(prediction.pose);
    msg.twist.twist.linear.x = prediction.velocity.x();
    msg.twist.twist.linear.y = prediction.velocity.y();
    msg.twist.twist.linear.z = prediction.velocity.z();
    imu_prediction_publisher_.publish(msg);
}

void LocLIO::InsCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    PublishInsTrajectory(*msg);
    latest_ins_pose_ = OdomToSophus(*msg);
    if (refine_ins_init_) return;
    if (initialized_from_ins_) return;
    odometry_.SetInitialPose(*latest_ins_pose_);
    path_msg_.poses.clear();
    path_msg_.header.frame_id = odom_frame_;
    initialized_from_ins_ = true;
    ROS_INFO_STREAM("LocLIO initialized from INS pose xyz="
                    << latest_ins_pose_->translation().transpose());
}

void LocLIO::PublishInsTrajectory(const nav_msgs::Odometry &odom) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = odom.header.stamp;
    // 这里假设 INS pose 和 LO 输出共用同一全局坐标系；launch 默认二者都是 map。
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = odom.pose.pose;

    ins_path_msg_.header.stamp = odom.header.stamp;
    ins_path_msg_.header.frame_id = odom_frame_;
    ins_path_msg_.poses.push_back(pose_msg);
    ins_traj_publisher_.publish(ins_path_msg_);
}

bool LocLIO::LoadGlobalMapIfNeeded(const std::optional<Eigen::Vector3d> &crop_center) {
    if (global_map_loaded_) return true;

    pcl::PointCloud<pcl::PointXYZI> cloud_i;
    ROS_INFO_STREAM("Loading global map for init/RViz: " << map_path_
                    << ", crop_radius=" << init_map_crop_radius_
                    << (crop_center ? ", crop_center=" + std::to_string(crop_center->x()) + "," +
                                          std::to_string(crop_center->y())
                                    : ", crop_center=none"));
    if (pcl::io::loadPCDFile(map_path_, cloud_i) != 0) {
        ROS_ERROR_STREAM("Failed to load global map: " << map_path_);
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZ>);
    raw_map->reserve(cloud_i.points.size());
    const bool crop_enabled = crop_center && init_map_crop_radius_ > 0.0;
    const double crop_radius2 = init_map_crop_radius_ * init_map_crop_radius_;
    size_t finite_point_count = 0;
    for (const auto &pt : cloud_i.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        ++finite_point_count;
        if (crop_enabled) {
            const double dx = static_cast<double>(pt.x) - crop_center->x();
            const double dy = static_cast<double>(pt.y) - crop_center->y();
            if (dx * dx + dy * dy > crop_radius2) continue;
        }
        raw_map->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }
    raw_map->width = static_cast<uint32_t>(raw_map->size());
    raw_map->height = 1;
    raw_map->is_dense = false;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(raw_map);
    voxel_filter.setLeafSize(init_ndt_map_voxel_size_,
                             init_ndt_map_voxel_size_,
                             init_ndt_map_voxel_size_);
    voxel_filter.filter(*ndt_target_map_);
    const size_t cropped_point_count = raw_map->size();
    raw_map.reset();

    global_map_points_.clear();
    if (publish_global_map_) {
        global_map_points_.reserve(ndt_target_map_->size());
    }

    global_map_voxel_.emplace(config_.voxel_size,
                              config_.max_range,
                              std::numeric_limits<double>::max(),
                              config_.planarity_threshold,
                              config_.max_points_per_voxel);
    constexpr size_t chunk_size = 200000;
    std::vector<Eigen::Vector3d> chunk;
    chunk.reserve(chunk_size);
    for (const auto &point : ndt_target_map_->points) {
        const Eigen::Vector3d eigen_point(point.x, point.y, point.z);
        if (publish_global_map_) {
            global_map_points_.emplace_back(eigen_point);
        }
        chunk.emplace_back(eigen_point);
        if (chunk.size() >= chunk_size) {
            global_map_voxel_->AddPoints(chunk);
            chunk.clear();
        }
    }
    if (!chunk.empty()) global_map_voxel_->AddPoints(chunk);

    global_map_loaded_ = cropped_point_count > 0 && !global_map_voxel_->Empty() && !ndt_target_map_->empty();
    if (global_map_loaded_) {
        global_map_crop_center_ = crop_center;
    }
    ROS_INFO_STREAM("Global map loaded. finite points=" << finite_point_count
                    << ", cropped points=" << cropped_point_count
                    << ", init map points=" << ndt_target_map_->size()
                    << ", voxel map points=" << global_map_voxel_->PointCount()
                    << ", voxel cells=" << global_map_voxel_->VoxelCount()
                    << ", map voxel leaf=" << init_ndt_map_voxel_size_
                    << ", crop_radius=" << init_map_crop_radius_);
    return global_map_loaded_;
}

void LocLIO::LoadAndPublishGlobalMap() {
    if (!LoadGlobalMapIfNeeded(global_map_crop_center_)) {
        return;
    }

    std_msgs::Header header;
    // 使用 0 时间戳作为静态地图，避免 rosbag /clock 下 RViz 因时间窗口丢显示。
    header.stamp = ros::Time(0);
    header.frame_id = global_map_frame_;
    global_map_msg_ = *EigenToPointCloud2(global_map_points_, header);
    global_map_publisher_.publish(*global_map_msg_);
    ROS_INFO_STREAM("Published RViz global map points: " << global_map_points_.size()
                    << ", frame: " << global_map_frame_);
}

void LocLIO::PublishGlobalMap(const ros::TimerEvent &) {
    if (global_map_msg_) {
        global_map_msg_->header.stamp = ros::Time(0);
        global_map_publisher_.publish(*global_map_msg_);
    }
}

std::tuple<Sophus::SE3d, double, size_t, size_t> LocLIO::FineAlignInitialPose(
    const std::vector<Eigen::Vector3d> &source,
    const Sophus::SE3d &initial_guess) const {
    genz_icp::Registration registration(config_.max_num_iterations, config_.convergence_criterion);
    registration.SetTerminalStatusEnabled(false);
    const auto &[pose, planar_points, non_planar_points] =
        registration.RegisterFrame(source,
                                   *global_map_voxel_,
                                   initial_guess,
                                   init_max_correspondence_distance_,
                                   init_kernel_);
    const size_t correspondences = planar_points.size() + non_planar_points.size();
    const double score = source.empty() ? 0.0 : static_cast<double>(correspondences) / static_cast<double>(source.size());
    return {pose, score, correspondences, source.size()};
}

Eigen::Matrix4f LocLIO::SophusToMatrix4f(const Sophus::SE3d &pose) const {
    return pose.matrix().cast<float>();
}

Sophus::SE3d LocLIO::Matrix4fToSophus(const Eigen::Matrix4f &matrix) const {
    Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond quat(rotation);
    quat.normalize();
    return Sophus::SE3d(quat, matrix.block<3, 1>(0, 3).cast<double>());
}

std::optional<Sophus::SE3d> LocLIO::CoarseAlignWithNdt(const std::vector<Eigen::Vector3d> &source,
                                                      const Sophus::SE3d &initial_guess) const {
    if (!global_map_loaded_ || ndt_target_map_->empty()) return std::nullopt;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    source_cloud->reserve(source.size());
    for (const auto &point : source) {
        source_cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
    source_cloud->width = static_cast<uint32_t>(source_cloud->size());
    source_cloud->height = 1;
    source_cloud->is_dense = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(source_cloud);
    voxel_filter.setLeafSize(init_ndt_source_voxel_size_,
                             init_ndt_source_voxel_size_,
                             init_ndt_source_voxel_size_);
    voxel_filter.filter(*source_filtered);

    if (source_filtered->empty()) return std::nullopt;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(init_ndt_transformation_epsilon_);
    ndt.setStepSize(init_ndt_step_size_);
    ndt.setResolution(init_ndt_resolution_);
    ndt.setMaximumIterations(init_ndt_max_iterations_);
    ndt.setInputSource(source_filtered);
    ndt.setInputTarget(ndt_target_map_);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    ndt.align(aligned, SophusToMatrix4f(initial_guess));
    const double fitness = ndt.getFitnessScore();
    ROS_INFO_STREAM("Init NDT coarse align: converged=" << ndt.hasConverged()
                    << ", fitness=" << fitness
                    << ", source=" << source_filtered->size());
    if (!ndt.hasConverged() || !std::isfinite(fitness) || fitness > init_ndt_max_fitness_score_) {
        return std::nullopt;
    }
    return Matrix4fToSophus(ndt.getFinalTransformation());
}

bool LocLIO::InitializeFromMatchedPose(const sensor_msgs::PointCloud2::ConstPtr &msg,
                                      const std::vector<Eigen::Vector3d> &points) {
    if (!latest_ins_pose_) {
        ROS_WARN_THROTTLE(2.0, "Waiting for INS pose before refined initialization");
        return false;
    }
    if (!LoadGlobalMapIfNeeded(latest_ins_pose_->translation())) {
        ROS_WARN("Refined init requested but global map is unavailable. Falling back to raw INS pose.");
        odometry_.SetInitialPose(*latest_ins_pose_);
        initialized_from_ins_ = true;
        return true;
    }
    if (publish_global_map_ && !global_map_msg_) {
        LoadAndPublishGlobalMap();
    }

    const auto cropped = genz_icp::Preprocess(points, config_.max_range, config_.min_range);
    const auto source = genz_icp::VoxelDownsample(cropped, init_scan_voxel_size_);
    if (source.empty()) {
        ROS_WARN("First cloud has no valid points after preprocessing; cannot initialize LO yet.");
        return false;
    }

    const auto [fine_pose, fine_score, fine_corr, fine_source] =
        FineAlignInitialPose(source, *latest_ins_pose_);
    ROS_INFO_STREAM("Init fine align from INS: score=" << fine_score
                    << ", corr=" << fine_corr << "/" << fine_source);

    Sophus::SE3d init_pose = fine_pose;
    bool accepted = fine_score >= init_fine_score_threshold_ &&
                    fine_corr >= static_cast<size_t>(init_min_correspondences_);
    std::string method = "INS + GenZ fine";

    if (!accepted && init_use_ndt_fallback_) {
        ROS_WARN_STREAM("Init fine score is low. Trying NDT coarse fallback. score="
                        << fine_score << ", threshold=" << init_fine_score_threshold_
                        << ", corr=" << fine_corr);
        const auto ndt_pose = CoarseAlignWithNdt(source, *latest_ins_pose_);
        if (ndt_pose) {
            const auto [refined_pose, refined_score, refined_corr, refined_source] =
                FineAlignInitialPose(source, *ndt_pose);
            ROS_INFO_STREAM("Init fine align after NDT: score=" << refined_score
                            << ", corr=" << refined_corr << "/" << refined_source);
            if (refined_score >= init_fine_score_threshold_ &&
                refined_corr >= static_cast<size_t>(init_min_correspondences_)) {
                init_pose = refined_pose;
                accepted = true;
                method = "NDT coarse + GenZ fine";
            } else {
                init_pose = refined_pose;
                method = "NDT coarse + GenZ fine, low score fallback";
            }
        } else {
            method = "raw INS fallback, NDT failed";
            init_pose = *latest_ins_pose_;
        }
    } else if (!accepted) {
        method = "raw INS fallback, fine score low and NDT disabled";
        init_pose = *latest_ins_pose_;
    }

    odometry_.SetInitialPose(init_pose);
    path_msg_.poses.clear();
    path_msg_.header.frame_id = odom_frame_;
    initialized_from_ins_ = true;
    ROS_INFO_STREAM("LocLIO initialized by " << method
                    << ", accepted=" << accepted
                    << ", xyz=" << init_pose.translation().transpose()
                    << ", cloud stamp=" << msg->header.stamp.toSec());
    return true;
}

Sophus::SE3d LocLIO::LookupTransform(const std::string &target_frame,
                                             const std::string &source_frame) const {
    // TF 查询失败时，这里保存失败原因，方便打印日志。
    std::string err_msg;
    // 先检查源坐标系、目标坐标系是否存在，并确认当前可以变换。
    if (tf2_buffer_._frameExists(source_frame) &&  //
        tf2_buffer_._frameExists(target_frame) &&  //
        tf2_buffer_.canTransform(target_frame, source_frame, ros::Time(0), &err_msg)) {
        try {
            // 查询最新可用的 target_frame <- source_frame 变换。
            auto tf = tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
            // 把 ROS TransformStamped 转成 Sophus::SE3d，供算法内部使用。
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            // 捕获 TF 查询异常，避免节点崩溃。
            ROS_WARN("%s", ex.what());
        }
    }
    // 走到这里说明无法取得 TF，打印源/目标坐标系和失败原因。
    ROS_WARN("Failed to find tf between %s and %s. Reason=%s", target_frame.c_str(),
             source_frame.c_str(), err_msg.c_str());
    // 返回单位变换作为兜底；这可能让坐标系不准，所以日志很重要。
    return {};
}

void LocLIO::RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // 记录当前点云的坐标系，例如 lidar、base_link。
    const auto cloud_frame_id = msg->header.frame_id;
    // 把 ROS 点云消息转成 Eigen 点数组，供 GenZ-ICP 处理。
    const auto points = PointCloud2ToEigen(msg);

    if (use_ins_init_ && !initialized_from_ins_) {
        if (refine_ins_init_) {
            if (!InitializeFromMatchedPose(msg, points)) return;
        } else {
            ROS_WARN_THROTTLE(2.0, "Waiting for INS init before processing point clouds");
            return;
        }
    }

    // 根据 deskew 参数决定是否读取每个点的相对时间戳。
    const auto timestamps = [&]() -> std::vector<double> {
        // 不启用 deskew 时返回空数组，算法会跳过运动补偿。
        if (!config_.deskew) return {};
        // 启用 deskew 时从 PointCloud2 字段 t/timestamp/time 中读取时间。
        return GetTimestamps(msg);
    }();
    // 如果没有指定 base_frame，或者 base_frame 和点云 frame 相同，就直接在点云坐标系估计。
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // 优先用 IMU 预积分给出本帧绝对初值；IMU 不可用时按参数回退。
    const auto imu_prediction = PredictPoseWithImu(msg->header.stamp);
    if (imu_prediction) {
        ++imu_prediction_success_count_;
        PublishImuPrediction(*imu_prediction, msg->header.stamp, cloud_frame_id);
    } else if (use_imu_prediction_ && last_corrected_pose_) {
        ++imu_prediction_fallback_count_;
        ROS_WARN_THROTTLE(2.0,
                          "IMU prediction unavailable, fallback count=%zu, success count=%zu",
                          imu_prediction_fallback_count_, imu_prediction_success_count_);
    }

    const auto registration_result = [&]() {
        if (imu_prediction) {
            return odometry_.RegisterFrame(points, timestamps, imu_prediction->pose);
        }
        if (use_imu_prediction_ && !imu_fallback_to_constant_velocity_ && last_corrected_pose_) {
            // 禁止匀速回退时使用上一帧校正 pose，相当于零运动初值。
            return odometry_.RegisterFrame(points, timestamps, *last_corrected_pose_);
        }
        return odometry_.RegisterFrame(points, timestamps);
    }();
    const auto &[planar_points, non_planar_points] = registration_result;

    // 取出最新一帧配准后的位姿；这是 LiDAR/点云坐标系下的里程计位姿。
    const Sophus::SE3d genz_pose = odometry_.poses().back();
    CorrectImuState(genz_pose, msg->header.stamp, imu_prediction);

    // 如果用户指定了 base_frame，需要把 LiDAR 运动转换到车体坐标系运动。
    const auto pose = [&]() -> Sophus::SE3d {
        // 点云 frame 和 base frame 一样时，不需要额外坐标变换。
        if (egocentric_estimation) return genz_pose;
        // 查询 base_frame <- cloud_frame_id 的外参。
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        // 用相似变换把 LiDAR 位姿转换成 base 位姿。
        return cloud2base * genz_pose * cloud2base.inverse();
    }();

    // 把当前位姿发布为 ROS Odometry、Path，以及可选 TF。
    PublishOdometry(pose, msg->header.stamp, cloud_frame_id);

    // 调试点云发布比较耗时，所以只在 visualize=true 时执行。
    if (publish_debug_clouds_) {
        // 发布局部地图、平面点和非平面点，方便 RViz 观察配准状态。
        PublishClouds(msg->header.stamp, cloud_frame_id, planar_points, non_planar_points);
    }
}

void LocLIO::PublishOdometry(const Sophus::SE3d &pose,
                                     const ros::Time &stamp,
                                     const std::string &cloud_frame_id) {
    // 如果配置要求发布 TF，就广播 odom_frame_ 到 base_frame_/cloud_frame 的变换。
    if (publish_odom_tf_) {
        // 创建 ROS TF 消息。
        geometry_msgs::TransformStamped transform_msg;
        // 使用点云时间戳，保证 TF 与点云同一时刻。
        transform_msg.header.stamp = stamp;
        // 父坐标系是里程计坐标系。
        transform_msg.header.frame_id = odom_frame_;
        // 子坐标系优先使用 base_frame；没有 base_frame 时使用点云 frame。
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        // 把 Sophus SE3 转成 ROS Transform。
        transform_msg.transform = tf2::sophusToTransform(pose);
        // 广播 TF。
        tf_broadcaster_.sendTransform(transform_msg);
    }

    // 构造当前帧的 PoseStamped，用于累计轨迹。
    geometry_msgs::PoseStamped pose_msg;
    // 轨迹点时间戳与点云时间戳一致。
    pose_msg.header.stamp = stamp;
    // 轨迹位于 odom_frame_ 坐标系。
    pose_msg.header.frame_id = odom_frame_;
    // 把 Sophus SE3 转成 ROS Pose。
    pose_msg.pose = tf2::sophusToPose(pose);
    // 追加到 Path 历史轨迹中。
    path_msg_.poses.push_back(pose_msg);
    // 发布完整 Path。
    traj_publisher_.publish(path_msg_);

    // 构造 nav_msgs/Odometry，用于算法输出和后续评估。
    nav_msgs::Odometry odom_msg;
    // Odometry 时间戳与点云时间戳一致。
    odom_msg.header.stamp = stamp;
    // Odometry 父坐标系是 odom_frame_。
    odom_msg.header.frame_id = odom_frame_;
    // 写入当前估计位姿；这里没有填速度和协方差。
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    // 发布 /genz/odometry。
    odom_publisher_.publish(odom_msg);
}

void LocLIO::PublishClouds(const ros::Time &stamp,
                                   const std::string &cloud_frame_id,
                                   const std::vector<Eigen::Vector3d> &planar_points,
                                   const std::vector<Eigen::Vector3d> &non_planar_points) {
    // 调试点云默认发布在 odom_frame_ 下。
    std_msgs::Header odom_header;
    // 使用当前点云帧时间戳。
    odom_header.stamp = stamp;
    // 设置点云输出坐标系。
    odom_header.frame_id = odom_frame_;

    // 取出 GenZ-ICP 当前维护的局部地图。
    const auto genz_map = odometry_.LocalMap();

    // 如果不发布 TF，RViz 中更像是一个以传感器为中心的调试世界。
    if (!publish_odom_tf_) {
        // 平面点和非平面点保留在原始点云坐标系。
        std_msgs::Header cloud_header;
        // 使用当前点云时间戳。
        cloud_header.stamp = stamp;
        // 使用点云原始 frame。
        cloud_header.frame_id = cloud_frame_id;

        // 发布局部地图；这里 header 是 odom_frame_。
        map_publisher_.publish(*EigenToPointCloud2(genz_map, odom_header));
        // 发布当前帧平面点；用于看点到面约束来源。
        planar_points_publisher_.publish(*EigenToPointCloud2(planar_points, cloud_header));
        // 发布当前帧非平面点；用于看点到点约束来源。
        non_planar_points_publisher_.publish(*EigenToPointCloud2(non_planar_points, cloud_header));

        // 不需要后面的 TF 坐标变换逻辑，直接返回。
        return;
    }

    // 如果发布 TF，理论上可以通过 TF 把点云放到正确位置。
    const auto cloud2odom = LookupTransform(odom_frame_, cloud_frame_id);
    // 平面点发布在 odom_frame_；注意这里没有显式用 cloud2odom 变换。
    planar_points_publisher_.publish(*EigenToPointCloud2(planar_points, odom_header));
    // 非平面点发布在 odom_frame_；注意这里没有显式用 cloud2odom 变换。
    non_planar_points_publisher_.publish(*EigenToPointCloud2(non_planar_points, odom_header));

    // 如果设置了 base_frame，需要把局部地图从 LiDAR 估计坐标转换到 base 坐标。
    if (!base_frame_.empty()) {
        // 查询 base_frame <- cloud_frame_id 的外参。
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        // 发布经过外参转换后的局部地图。
        map_publisher_.publish(*EigenToPointCloud2(genz_map, cloud2base, odom_header));
    } else {
        // 没有 base_frame 时直接发布局部地图。
        map_publisher_.publish(*EigenToPointCloud2(genz_map, odom_header));
    }
}

}  // namespace genz_icp_ros

int main(int argc, char **argv) {
    // 初始化 ROS 节点，节点名为 genz_icp。
    ros::init(argc, argv, "genz_icp");
    // 公共 NodeHandle，用于普通话题。
    ros::NodeHandle nh;
    // 私有 NodeHandle，用于读取 ~param。
    ros::NodeHandle nh_private("~");

    // 构造并启动里程计服务器；构造函数里完成订阅和发布初始化。
    genz_icp_ros::LocLIO node(nh, nh_private);

    // 进入 ROS 回调循环，持续处理点云。
    ros::spin();

    // 正常退出。
    return 0;
}
