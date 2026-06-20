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

#include <optional>
#include <string>
#include <vector>

// ROS
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace genz_icp_ros {

class LocLO {
public:
    /// ROS1 里程计节点构造函数：读取参数、初始化 GenZ-ICP、建立订阅和发布。
    LocLO(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

private:
    /// INS 回调：首帧 INS 作为 LO 初始位姿，后续只缓存参考位姿。
    void InsCallback(const nav_msgs::Odometry::ConstPtr &msg);

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

    /// 定时重发全局地图，避免 RViz 晚启动或显示临时丢失。
    void PublishGlobalMap(const ros::TimerEvent &event);

    /// 把 ROS Odometry 转成 Sophus SE3。
    Sophus::SE3d OdomToSophus(const nav_msgs::Odometry &odom) const;

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
    /// 是否等待首帧 INS 作为初始位姿。
    bool use_ins_init_{true};
    /// 是否已经用 INS 初始化 GenZ-ICP。
    bool initialized_from_ins_{false};
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

    /// 里程计发布器，输出 /genz/odometry。
    ros::Publisher odom_publisher_;
    /// 局部地图发布器，输出 /genz/local_map。
    ros::Publisher map_publisher_;
    /// 轨迹发布器，输出 /genz/trajectory。
    ros::Publisher traj_publisher_;
    /// INS 参考轨迹发布器，输出 /genz/ins_trajectory。
    ros::Publisher ins_traj_publisher_;
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
