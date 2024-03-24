/*
MIT License

Copyright (c) 2019 Michael Kösel

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

#include <limits>
#include <memory>

#include "dogm_ros/dogm_node.h"

namespace dogm_ros {

DOGM_Node::DOGM_Node(const std::string& node_name, bool show_debug)
    : Node(node_name), show_debug(show_debug), grid_map_(nullptr), cumulative_time(0), map_count(0), pos_x(0), pos_y(0),
      pos_yaw(0)
{

    this->declare_parameter("subscribe/laser_topic", std::string("/scan"));
    std::string subscribe_laser_topic = this->get_parameter("subscribe/laser_topic").as_string();

    this->declare_parameter("subscribe/odometry_topic", std::string("/odometry/filtered"));
    std::string subscribe_odometry_topic = this->get_parameter("subscribe/odometry_topic").as_string();

    this->declare_parameter("publish/dogm_topic", std::string("/dogm/map"));
    std::string publish_dogm_topic = this->get_parameter("publish/dogm_topic").as_string();

    this->declare_parameter("publish/occ_topic", std::string("/dogm/occ"));
    std::string publish_occ_topic = this->get_parameter("publish/occ_topic").as_string();

    this->declare_parameter("map.size", 20.0f);
    params_.size = this->get_parameter("map.size").as_double();

    this->declare_parameter("map.resolution", 0.2f);
    params_.resolution = this->get_parameter("map.resolution").as_double();

    this->declare_parameter("particles.particle_count", 500000);
    params_.particle_count = this->get_parameter("particles.particle_count").as_int();

    this->declare_parameter("particles.new_born_particle_count", 100000);
    params_.new_born_particle_count = this->get_parameter("particles.new_born_particle_count").as_int();

    this->declare_parameter("particles.persistence_probability", 0.7f);
    params_.persistence_prob = this->get_parameter("particles.persistence_probability").as_double();

    this->declare_parameter("particles.process_noise_position", 0.1f);
    params_.stddev_process_noise_position = this->get_parameter("particles.process_noise_position").as_double();

    this->declare_parameter("particles.process_noise_velocity", 1.0f);
    params_.stddev_process_noise_velocity = this->get_parameter("particles.process_noise_velocity").as_double();

    this->declare_parameter("particles.birth_probability", 0.5f);
    params_.birth_prob = this->get_parameter("particles.birth_probability").as_double();

    this->declare_parameter("particles.velocity_persistent", 5.0f);
    params_.stddev_velocity = this->get_parameter("particles.velocity_persistent").as_double();

    this->declare_parameter("particles.velocity_birth", 5.0f);
    params_.init_max_velocity = this->get_parameter("particles.velocity_birth").as_double();

    this->declare_parameter("laser.fov", 360.0f);
    laser_params_.fov = this->get_parameter("laser.fov").as_double();

    this->declare_parameter("laser.angle_increment", 0.0087f);
    float lidar_inc = this->get_parameter("laser.angle_increment").as_double();

    this->declare_parameter("laser.max_range", 30.0f);
    laser_params_.max_range = this->get_parameter("laser.max_range").as_double();
    this->declare_parameter("laser.sigma", 0.1f);
    laser_params_.stddev_range = this->get_parameter("laser.sigma").as_double();

    // transforms
    this->declare_parameter("tf.base", std::string("base_link"));
    base_frame = this->get_parameter("tf.base").as_string();
    this->declare_parameter("tf.lidar", std::string("velodyne"));
    lidar_frame = this->get_parameter("tf.lidar").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    laser_params_.resolution = 0.1;
    laser_params_.angle_increment = lidar_inc * 180.0 / M_PI;

    laser_conv_ = std::make_unique<dogm::LaserMeasurementGrid>(laser_params_, params_.size, params_.resolution);
    grid_map_ = std::make_unique<dogm::DOGM>(params_);

    is_first_measurement_ = true;
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        subscribe_laser_topic, 1, std::bind(&DOGM_Node::processLaserScan, this, std::placeholders::_1));
    subscriber_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
        subscribe_odometry_topic, 1, std::bind(&DOGM_Node::processOdometry, this, std::placeholders::_1));

    publisher_dogm_ = this->create_publisher<dogm_msgs::msg::DynamicOccupancyGrid>(publish_dogm_topic, 1);
    publisher_occ_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(publish_occ_topic, 1);

    if (show_debug) {
        // create debug windows so we can see what's going on locally
        //    namedWindow("Predicted", cv::WINDOW_NORMAL); // Create Window
        namedWindow("Born", cv::WINDOW_NORMAL);  // Create Window
        //    namedWindow("Persistent", cv::WINDOW_NORMAL); // Create Window
        //    namedWindow("Occupancy", cv::WINDOW_NORMAL); // Create Window
        namedWindow("Particles!", cv::WINDOW_NORMAL);  // Create Window
        namedWindow("Free Mass", cv::WINDOW_NORMAL);   // Create Window
        namedWindow("Occ Mass", cv::WINDOW_NORMAL);    // Create Window

        cv::startWindowThread();
    }
}

void DOGM_Node::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto time_stamp = rclcpp::Time(scan->header.stamp);
    auto data = scan->ranges;

    auto start = std::chrono::high_resolution_clock::now();

    if (pos_yaw == std::numeric_limits<float>::quiet_NaN()) {
        return;
    }

    auto cell_data = laser_conv_->generateGrid(data, pos_yaw * 180.0 / M_PI);

    {
        std::lock_guard<std::mutex> guard(grid_mutex);
#if USE_SOA
        if (!is_first_measurement_) {
            float dt = float((time_stamp - last_scan_update).nanoseconds()) / 1e9;
            grid_map_->updateGrid(cell_data, pos_x, pos_y, dt);
        } else {
            grid_map_->updateGrid(cell_data, pos_x, pos_y, 0.0);
            is_first_measurement_ = false;
        }
#else
        if (!is_first_measurement_) {
            float dt = float((time_stamp - last_scan_update).nanoseconds()) / 1e9;
            grid_map_->updateGrid(cell_data, pos_x, pos_y, dt, true);
        } else {
            grid_map_->updateGrid(cell_data, pos_x, pos_y, 0.0, true);
            is_first_measurement_ = false;
        }
#endif
    }
    last_scan_update = time_stamp;

    if (show_debug) {
        map_count++;

        // Track the execution time
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        cumulative_time += duration.count();

        auto img = getMeasuredOccMassImage();
        imshow("Occ Mass", img);
        img = getMeasuredFreeMassImage();
        imshow("Free Mass", img);
        if (map_count && map_count % 100) {
            std::cout << "After " << map_count << " iterations: " << double(cumulative_time) / map_count / 1000.0
                      << "mS per iteration";
        }
    }
}

void DOGM_Node::publishDynamicGrid()
{
    std::lock_guard<std::mutex> guard(grid_mutex);

    dogm_msgs::msg::DynamicOccupancyGrid dogma_msg;
    dogm_ros::DOGMRosConverter::toDOGMMessage(this->now(), *grid_map_, lidar_frame, dogma_msg);
    publisher_dogm_->publish(dogma_msg);
}

void DOGM_Node::publishOccupancyGrid()
{
    std::lock_guard<std::mutex> guard(grid_mutex);

    nav_msgs::msg::OccupancyGrid message;
    dogm_ros::DOGMRosConverter::toOccupancyGridMessage(this->now(), *grid_map_, lidar_frame, message);
    publisher_occ_->publish(message);
}

Eigen::MatrixXf DOGM_Node::getOccupancyGrid()
{
    auto size = grid_map_->getGridSize();
    auto grid_cells = grid_map_->getGridCells();
    Eigen::MatrixXf occ(size, size);

    std::lock_guard<std::mutex> guard(grid_mutex);
#pragma omp parallel for
    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            float free_mass = grid_cells.free_mass[y * size + x];
            float occ_mass = grid_cells.occ_mass[y * size + x];

            occ(y, x) = occ_mass + 0.5f * (1.0f - occ_mass - free_mass);
        }
    }

    grid_map_->freeGridCells(grid_cells);
    return occ;
}

void DOGM_Node::processOdometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{

    auto time_stamp = rclcpp::Time(odom_msg->header.stamp);

    geometry_msgs::msg::Pose tf_pose;

    try {
        // geometry_msgs::msg::TransformStamped t;
        auto transform = tf_buffer_->lookupTransform("map", "odom", time_stamp);
        tf2::doTransform(odom_msg->pose.pose, tf_pose, transform);

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "%s -- Unable to correct Odometry, going with what we have", ex.what());
        tf_pose = odom_msg->pose.pose;
    }

    tf2::Quaternion q;

    q.setW(tf_pose.orientation.w);
    q.setX(tf_pose.orientation.x);
    q.setY(tf_pose.orientation.y);
    q.setZ(tf_pose.orientation.z);
    auto mat = tf2::Matrix3x3(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    pos_x = tf_pose.position.x;
    pos_y = tf_pose.position.y;
    pos_yaw = float(yaw);
}

cv::Mat DOGM_Node::getMeasuredOccMassImage() const
{
    auto grid_size = int(params_.size / params_.resolution);
    auto host_cell_data = grid_map_->getMeasurementCells();

    cv::Mat image(grid_size, grid_size, CV_8UC3);
    for (int i = 0; i < host_cell_data.size; i++) {
        cv::Vec3b color;
        int x = i % grid_size;
        int y = i / grid_size;
        color[0] = color[1] = color[2] = uchar((1.0 - host_cell_data.occ_mass[i]) * 255);
        image.at<cv::Vec3b>(grid_size - x - 1, grid_size - y - 1) = color;
    }
    return image;
}

cv::Mat DOGM_Node::getMeasuredFreeMassImage() const
{
    auto grid_size = int(params_.size / params_.resolution);
    auto host_cell_data = grid_map_->getMeasurementCells();

    cv::Mat image(grid_size, grid_size, CV_8UC3);
    for (int i = 0; i < host_cell_data.size; i++) {
        cv::Vec3b color;
        int x = i % grid_size;
        int y = i / grid_size;
        color[0] = color[1] = color[2] = uchar((1.0 - host_cell_data.free_mass[i]) * 255);
        image.at<cv::Vec3b>(grid_size - x - 1, grid_size - y - 1) = color;
    }
    return image;
}

}  // namespace dogm_ros
