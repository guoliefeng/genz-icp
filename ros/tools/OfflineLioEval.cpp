// MIT License
// 离线回放 LocLIO：不依赖 ROS master，便于在自动化环境中复现完整轨迹。

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sophus/se3.hpp>

#include "../ros1/Utils.hpp"
#include "genz_icp/pipeline/GenZICP.hpp"

namespace {

struct ImuSample {
    ros::Time stamp;
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
};

struct ImuPrediction {
    Sophus::SE3d pose;
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
};

Sophus::SE3d OdomToSophus(const nav_msgs::Odometry &odom) {
    const auto &p = odom.pose.pose.position;
    const auto &q = odom.pose.pose.orientation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    quat.normalize();
    return Sophus::SE3d(quat, Eigen::Vector3d(p.x, p.y, p.z));
}

nav_msgs::Odometry SophusToOdom(const Sophus::SE3d &pose,
                                const ros::Time &stamp,
                                const std::string &child_frame) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = child_frame;
    odom.pose.pose = tf2::sophusToPose(pose);
    return odom;
}

class OfflineLocLio {
public:
    explicit OfflineLocLio(bool use_imu_prediction)
        : odometry_(MakeConfig()), use_imu_prediction_(use_imu_prediction) {
        odometry_.SetTerminalStatusEnabled(false);
    }

    void AddIns(const nav_msgs::Odometry &msg) {
        if (initialized_) return;
        odometry_.SetInitialPose(OdomToSophus(msg));
        initialized_ = true;
        std::cout << "Initialized from INS at " << msg.header.stamp.toSec() << '\n';
    }

    void AddImu(const sensor_msgs::Imu &msg) {
        const Eigen::Vector3d gyro(msg.angular_velocity.x,
                                   msg.angular_velocity.y,
                                   msg.angular_velocity.z);
        const Eigen::Vector3d accel(msg.linear_acceleration.x,
                                    msg.linear_acceleration.y,
                                    msg.linear_acceleration.z);
        if (!gyro.allFinite() || !accel.allFinite()) return;
        imu_buffer_.push_back({msg.header.stamp, gyro, accel * kAccelScale});
        const ros::Time keep_after = msg.header.stamp - ros::Duration(5.0);
        while (imu_buffer_.size() > 2 && imu_buffer_[1].stamp < keep_after) {
            imu_buffer_.pop_front();
        }
    }

    std::optional<nav_msgs::Odometry> AddCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        if (!initialized_) return std::nullopt;

        const auto prediction = use_imu_prediction_ ? Predict(msg->header.stamp) : std::nullopt;
        const auto points = genz_icp_ros::utils::PointCloud2ToEigen(msg);
        if (prediction) {
            odometry_.RegisterFrame(points, {}, prediction->pose);
            ++prediction_count_;
        } else {
            odometry_.RegisterFrame(points, {});
            if (use_imu_prediction_ && last_pose_) ++fallback_count_;
        }

        const Sophus::SE3d pose = odometry_.poses().back();
        Correct(pose, msg->header.stamp, prediction);
        ++cloud_count_;
        return SophusToOdom(pose, msg->header.stamp, msg->header.frame_id);
    }

    size_t cloud_count() const { return cloud_count_; }
    size_t prediction_count() const { return prediction_count_; }
    size_t fallback_count() const { return fallback_count_; }

private:
    static genz_icp::pipeline::GenZConfig MakeConfig() {
        genz_icp::pipeline::GenZConfig config;
        config.deskew = false;
        config.max_range = 100.0;
        config.min_range = 0.5;
        config.voxel_size = 0.6;
        config.desired_num_voxelized_points = 3000;
        config.planarity_threshold = 0.2;
        config.max_points_per_voxel = 3;
        config.initial_threshold = 2.0;
        config.min_motion_th = 0.1;
        config.max_num_iterations = 100;
        config.convergence_criterion = 0.0001;
        return config;
    }

    std::optional<ImuPrediction> Predict(const ros::Time &target_stamp) const {
        if (!last_pose_ || !last_stamp_ || imu_buffer_.empty()) return std::nullopt;
        const double prediction_interval = (target_stamp - *last_stamp_).toSec();
        if (prediction_interval <= 0.0 || prediction_interval > kMaxPredictionInterval) {
            return std::nullopt;
        }

        const ImuSample *measurement = nullptr;
        for (const auto &sample : imu_buffer_) {
            if (sample.stamp <= *last_stamp_) measurement = &sample;
            else break;
        }
        if (!measurement) {
            measurement = &imu_buffer_.front();
            if ((measurement->stamp - *last_stamp_).toSec() > kMaxSampleGap) {
                return std::nullopt;
            }
        }

        Sophus::SO3d rotation = last_pose_->so3();
        Eigen::Vector3d position = last_pose_->translation();
        Eigen::Vector3d velocity = velocity_;
        ros::Time integration_stamp = *last_stamp_;

        auto integrate = [&](const Eigen::Vector3d &gyro,
                             const Eigen::Vector3d &accel,
                             double dt) {
            if (dt <= 0.0) return true;
            if (dt > kMaxSampleGap || gyro.norm() > kMaxGyro || accel.norm() > kMaxAccel) {
                return false;
            }
            const Sophus::SO3d half_rotation =
                rotation * Sophus::SO3d::exp(0.5 * gyro * dt);
            const Eigen::Vector3d world_acceleration =
                half_rotation * accel + Eigen::Vector3d(0.0, 0.0, -kGravity);
            position += velocity * dt + 0.5 * world_acceleration * dt * dt;
            velocity += world_acceleration * dt;
            rotation = rotation * Sophus::SO3d::exp(gyro * dt);
            return position.allFinite() && velocity.allFinite();
        };

        for (const auto &sample : imu_buffer_) {
            if (sample.stamp <= *last_stamp_) continue;
            if (sample.stamp > target_stamp) break;
            const double dt = (sample.stamp - integration_stamp).toSec();
            const Eigen::Vector3d gyro =
                0.5 * (measurement->angular_velocity + sample.angular_velocity);
            const Eigen::Vector3d accel =
                0.5 * (measurement->linear_acceleration + sample.linear_acceleration);
            if (!integrate(gyro, accel, dt)) return std::nullopt;
            measurement = &sample;
            integration_stamp = sample.stamp;
        }

        if (!integrate(measurement->angular_velocity,
                       measurement->linear_acceleration,
                       (target_stamp - integration_stamp).toSec())) {
            return std::nullopt;
        }
        return ImuPrediction{Sophus::SE3d(rotation, position), velocity};
    }

    void Correct(const Sophus::SE3d &pose,
                 const ros::Time &stamp,
                 const std::optional<ImuPrediction> &prediction) {
        if (!last_pose_ || !last_stamp_) {
            last_pose_ = pose;
            last_stamp_ = stamp;
            velocity_.setZero();
            return;
        }
        const double dt = (stamp - *last_stamp_).toSec();
        if (dt > 0.0 && dt <= kMaxPredictionInterval) {
            // 在线节点默认 correction gain=1，速度完全由相邻 ICP 位移校正。
            velocity_ = (pose.translation() - last_pose_->translation()) / dt;
        } else {
            velocity_.setZero();
        }
        last_pose_ = pose;
        last_stamp_ = stamp;
        while (imu_buffer_.size() > 2 && imu_buffer_[1].stamp <= stamp) {
            imu_buffer_.pop_front();
        }
    }

    static constexpr double kAccelScale = 9.80665;
    static constexpr double kGravity = 9.80665;
    static constexpr double kMaxSampleGap = 0.05;
    static constexpr double kMaxPredictionInterval = 0.25;
    static constexpr double kMaxAccel = 30.0;
    static constexpr double kMaxGyro = 3.0;

    genz_icp::pipeline::GenZICP odometry_;
    bool use_imu_prediction_{true};
    bool initialized_{false};
    std::deque<ImuSample> imu_buffer_;
    std::optional<Sophus::SE3d> last_pose_;
    std::optional<ros::Time> last_stamp_;
    Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
    size_t cloud_count_{0};
    size_t prediction_count_{0};
    size_t fallback_count_{0};
};

}  // namespace

int main(int argc, char **argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: offline_lio_eval INPUT.bag OUTPUT.bag [--constant-velocity]\n";
        return 2;
    }

    const bool use_imu_prediction = argc != 4 || std::string(argv[3]) != "--constant-velocity";

    rosbag::Bag input;
    rosbag::Bag output;
    try {
        input.open(argv[1], rosbag::bagmode::Read);
        output.open(argv[2], rosbag::bagmode::Write);
    } catch (const rosbag::BagException &e) {
        std::cerr << "Cannot open bag: " << e.what() << '\n';
        return 1;
    }

    OfflineLocLio lio(use_imu_prediction);
    std::cout << "Prediction model: "
              << (use_imu_prediction ? "IMU preintegration" : "constant velocity") << '\n';
    const std::vector<std::string> topics{
        "/localization/ins", "/ins_driver/imu", "/lidar_preprocessor/meta_cloud"};
    rosbag::View view(input, rosbag::TopicQuery(topics));

    for (const rosbag::MessageInstance &message : view) {
        const std::string topic = message.getTopic();
        if (topic == "/localization/ins") {
            if (const auto msg = message.instantiate<nav_msgs::Odometry>()) {
                lio.AddIns(*msg);
                output.write("/localization/ins", msg->header.stamp, *msg);
            }
        } else if (topic == "/ins_driver/imu") {
            if (const auto msg = message.instantiate<sensor_msgs::Imu>()) lio.AddImu(*msg);
        } else if (topic == "/lidar_preprocessor/meta_cloud") {
            if (const auto msg = message.instantiate<sensor_msgs::PointCloud2>()) {
                const auto odom = lio.AddCloud(msg);
                if (odom) output.write("/genz/odometry", odom->header.stamp, *odom);
                if (lio.cloud_count() % 100 == 0) {
                    std::cout << "Processed clouds: " << lio.cloud_count() << '\n';
                }
            }
        }
    }

    input.close();
    output.close();
    std::cout << "Done. clouds=" << lio.cloud_count()
              << ", imu_predictions=" << lio.prediction_count()
              << ", fallbacks=" << lio.fallback_count() << '\n';
    return 0;
}
