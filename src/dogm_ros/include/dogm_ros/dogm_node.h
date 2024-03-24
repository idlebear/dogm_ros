/*
MIT License

Copyright (c) 2019 Michael Kösel
Copyright (c) 2023 Barry Gilhuly

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#pragma once

#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include <dogm/mapping/laser_to_meas_grid.h>
#include <dogm_msgs/msg/dynamic_occupancy_grid.hpp>

#include <Eigen/Dense>
#include <mutex>

#include <opencv2/core/cuda.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "dogm_ros/dogm_ros.h"
#include "dogm_ros/state_estimate.h"
#include <utility>

namespace dogm_ros {

class DOGM_Node : public rclcpp::Node {
public:
    DOGM_Node(const std::string& node_name, bool show_debug);
    virtual ~DOGM_Node() = default;

    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void processOdometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    Eigen::MatrixXf getOccupancyGrid();
    void publishDynamicGrid();
    void publishOccupancyGrid();

    inline std::pair<float, float> getPos() { return std::make_pair(pos_x, pos_y); };

    inline float getOrientation() { return pos_yaw; };

protected:
    cv::Mat getMeasuredOccMassImage() const;
    cv::Mat getMeasuredFreeMassImage() const;

private:
    bool show_debug;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_laser_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odometry_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber_tf_;
    rclcpp::Publisher<dogm_msgs::msg::DynamicOccupancyGrid>::SharedPtr publisher_dogm_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_occ_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    dogm::DOGM::Params params_;
    dogm::LaserMeasurementGrid::Params laser_params_;

    rclcpp::Time last_scan_update;
    bool is_first_measurement_;
    long cumulative_time;
    int map_count;

    std::unique_ptr<dogm::LaserMeasurementGrid> laser_conv_;
    std::unique_ptr<dogm::DOGM> grid_map_;

    std::string base_frame;
    std::string lidar_frame;

    float pos_x;
    float pos_y;
    float pos_yaw;

    std::mutex grid_mutex;
};

}  // namespace dogm_ros
