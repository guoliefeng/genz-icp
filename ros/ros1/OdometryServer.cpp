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
#include <memory>
#include <utility>
#include <vector>

// GenZ-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 1 headers
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace genz_icp_ros {

// 把 Utils.hpp 里的点云转换函数引入当前命名空间，代码更短。
using utils::EigenToPointCloud2;
// 获取 PointCloud2 中每个点的相对时间戳，用于运动畸变补偿。
using utils::GetTimestamps;
// 把 ROS PointCloud2 转成 GenZ-ICP 使用的 Eigen::Vector3d 点数组。
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf2_listener_(tf2_ros::TransformListener(tf2_buffer_)) {
    // 读取车体坐标系名称；为空时表示直接使用点云坐标系。
    pnh_.param("base_frame", base_frame_, base_frame_);
    // 读取里程计输出坐标系名称，默认 odom。
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
    // 读取是否发布 odom->base 的 TF。
    pnh_.param("publish_odom_tf", publish_odom_tf_, false);
    // 读取是否发布调试点云，RViz 调试时使用。
    pnh_.param("visualize", publish_debug_clouds_, publish_debug_clouds_);
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

    // 订阅点云话题；launch 会把 pointcloud_topic remap 成真实 LiDAR topic。
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud_topic", queue_size_,
                                                              &OdometryServer::RegisterFrame, this);

    // 发布 GenZ-ICP 估计的里程计。
    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("/genz/odometry", queue_size_);
    // 发布累计轨迹 Path。
    traj_publisher_ = pnh_.advertise<nav_msgs::Path>("/genz/trajectory", queue_size_);
    // 只有 visualize=true 时才创建调试点云发布器，避免不必要的开销。
    if (publish_debug_clouds_) {
        // 发布 GenZ-ICP 在线维护的局部地图。
        map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/local_map", queue_size_);
        // 发布当前帧中用于点到面约束的平面点。
        planar_points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/planar_points", queue_size_);
        // 发布当前帧中用于点到点约束的非平面点。
        non_planar_points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/non_planar_points", queue_size_);
    }
    // TF buffer 使用独立线程，避免查询 TF 时阻塞主点云回调。
    tf2_buffer_.setUsingDedicatedThread(true);
    // Path 的坐标系固定为 odom_frame_。
    path_msg_.header.frame_id = odom_frame_;

    // 节点初始化完成。
    ROS_INFO("GenZ-ICP ROS 1 Odometry Node Initialized");
}

Sophus::SE3d OdometryServer::LookupTransform(const std::string &target_frame,
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

void OdometryServer::RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    // 记录当前点云的坐标系，例如 lidar、base_link。
    const auto cloud_frame_id = msg->header.frame_id;
    // 把 ROS 点云消息转成 Eigen 点数组，供 GenZ-ICP 处理。
    const auto points = PointCloud2ToEigen(msg);
    // 根据 deskew 参数决定是否读取每个点的相对时间戳。
    const auto timestamps = [&]() -> std::vector<double> {
        // 不启用 deskew 时返回空数组，算法会跳过运动补偿。
        if (!config_.deskew) return {};
        // 启用 deskew 时从 PointCloud2 字段 t/timestamp/time 中读取时间。
        return GetTimestamps(msg);
    }();
    // 如果没有指定 base_frame，或者 base_frame 和点云 frame 相同，就直接在点云坐标系估计。
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // 调用 GenZ-ICP 主入口：完成预处理、预测、ICP 配准、局部地图更新。
    const auto &[planar_points, non_planar_points] = odometry_.RegisterFrame(points, timestamps);

    // 取出最新一帧配准后的位姿；这是 LiDAR/点云坐标系下的里程计位姿。
    const Sophus::SE3d genz_pose = odometry_.poses().back();

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

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
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

void OdometryServer::PublishClouds(const ros::Time &stamp,
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
    genz_icp_ros::OdometryServer node(nh, nh_private);

    // 进入 ROS 回调循环，持续处理点云。
    ros::spin();

    // 正常退出。
    return 0;
}
