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

    DOGMRos::DOGMRos(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, bool show_debug)
            : show_debug(show_debug), nh_(nh), private_nh_(private_nh), grid_map_(nullptr),
              cumulative_time( 0 ), map_count(0) {
        std::string subscribe_laser_topic;

        private_nh_.param("subscribe/laser_topic", subscribe_laser_topic,
                          std::string("/scan"));
        std::string subscribe_odometry_topic;
        private_nh_.param("subscribe/odometry_topic", subscribe_odometry_topic,
                          std::string("/carla/ego_vehicle/odometry"));

        std::string publish_dogm_topic;
        private_nh_.param("publish/dogm_topic", publish_dogm_topic,
                          std::string("/dogm/map"));
        std::string publish_occ_topic;
        private_nh_.param("publish/occ_topic", publish_occ_topic,
                          std::string("/dogm/occ"));

        private_nh_.param("map/size", params_.size, 20.0f);
        private_nh_.param("map/resolution", params_.resolution, 0.1f);
        private_nh_.param("particles/particle_count", params_.particle_count, 500000);
        private_nh_.param("particles/new_born_particle_count",
                          params_.new_born_particle_count, 100000);
        private_nh_.param("particles/persistence_probability",
                          params_.persistence_prob, 0.7f);
        private_nh_.param("particles/process_noise_position",
                          params_.stddev_process_noise_position, 0.1f);
        private_nh_.param("particles/process_noise_velocity",
                          params_.stddev_process_noise_velocity, 1.0f);
        private_nh_.param("particles/birth_probability", params_.birth_prob, 0.5f);
        private_nh_.param("particles/velocity_persistent", params_.stddev_velocity,
                          5.0f);
        private_nh_.param("particles/velocity_birth", params_.init_max_velocity,
                          5.0f);

        private_nh_.param("laser/fov", laser_params_.fov, 360.0f);

        float lidar_inc;
        private_nh_.param("laser/angle_increment", lidar_inc, 0.0087f);
        private_nh_.param("laser/max_range", laser_params_.max_range, 10.0f);
        private_nh_.param("laser/sigma", laser_params_.stddev_range, 0.1f);

        private_nh_.param("tf/base", base_frame, std::string("base_link"));
        private_nh_.param("tf/lidar", lidar_frame, std::string("lidar"));

        laser_params_.resolution = 0.1;
        laser_params_.angle_increment = lidar_inc * 180.0 / M_PI;

        laser_conv_ = std::make_unique<dogm::LaserMeasurementGrid>(
                laser_params_, params_.size, params_.resolution);
        grid_map_ = std::make_unique<dogm::DOGM>(params_);

        is_first_measurement_ = true;
        subscriber_laser_ = nh_.subscribe(subscribe_laser_topic, 1, &DOGMRos::processLaserScan, this);
        subscriber_odometry_ = nh_.subscribe(subscribe_odometry_topic, 1, &DOGMRos::processOdometry, this);
        publisher_dogm_ =nh_.advertise<dogm_msgs::DynamicOccupancyGrid>(publish_dogm_topic, 1);
        publisher_occ_ = nh_.advertise<nav_msgs::OccupancyGrid>(publish_occ_topic, 1);

        if( show_debug ) {
            // create debug windows so we can see what's going on locally
            //    namedWindow("Predicted", cv::WINDOW_NORMAL); // Create Window
            namedWindow("Born", cv::WINDOW_NORMAL); // Create Window
            //    namedWindow("Persistent", cv::WINDOW_NORMAL); // Create Window
            //    namedWindow("Occupancy", cv::WINDOW_NORMAL); // Create Window
            namedWindow("Particles!", cv::WINDOW_NORMAL); // Create Window
            namedWindow("Free Mass", cv::WINDOW_NORMAL);  // Create Window
            namedWindow("Occ Mass", cv::WINDOW_NORMAL);   // Create Window

            cv::startWindowThread();
        }
    }


    void DOGMRos::processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan) {
        auto time_stamp = scan->header.stamp.toSec();
        auto data = scan->ranges;

        auto start = std::chrono::high_resolution_clock::now();

        if (pos_yaw == std::numeric_limits<float>::quiet_NaN()) {
            return;
        }

        auto cell_data = laser_conv_->generateGrid(data); // pos_yaw*180.0/M_PI );

        {
            std::lock_guard<std::mutex> guard( grid_mutex );
#if USE_SOA
            if (!is_first_measurement_) {
                auto dt = float(time_stamp - last_scan_update);
                grid_map_->updateGrid(cell_data, pos_x, pos_y, dt);
            } else {
                grid_map_->updateGrid(cell_data, pos_x, pos_y, 0.0);
                is_first_measurement_ = false;
            }
#else
            if (!is_first_measurement_) {
          float dt = time_stamp - last_time_stamp_;
          grid_map_->updateGrid(cell_data, pos_x, pos_y, dt, true);
        } else {
          grid_map_->updateGrid(cell_data, pos_x, pos_y, 0.0, true);
          is_first_measurement_ = false;
        }
#endif
        }
        last_scan_update = time_stamp;

        if( show_debug ) {
            map_count++;

            // Track the execution time
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            cumulative_time += duration.count();

            auto img = getMeasuredOccMassImage();
            imshow("Occ Mass", img);
            img = getMeasuredFreeMassImage();
            imshow("Free Mass", img);
            if( map_count  && map_count % 100 ) {
                std::cout << "After " << map_count << " iterations: " << double(cumulative_time) / map_count / 1000.0 << "mS per iteration";
            }
        }
    }

    void DOGMRos::publishDynamicGrid() {
        std::lock_guard<std::mutex> guard( grid_mutex );

        dogm_msgs::DynamicOccupancyGrid dogma_msg;
        dogm_ros::DOGMRosConverter::toDOGMMessage(*grid_map_, lidar_frame, dogma_msg );
        publisher_dogm_.publish(dogma_msg);
    }

    void DOGMRos::publishOccupancyGrid() {
        std::lock_guard<std::mutex> guard( grid_mutex );

        nav_msgs::OccupancyGrid message;
        dogm_ros::DOGMRosConverter::toOccupancyGridMessage(*grid_map_, lidar_frame, message );
        publisher_occ_.publish(message);
    }

    Eigen::MatrixXf DOGMRos::getOccupancyGrid() {
        auto size = grid_map_->getGridSize();
        auto grid_cells = grid_map_->getGridCells();
        Eigen::MatrixXf occ( size, size );

        std::lock_guard<std::mutex> guard( grid_mutex );
#pragma omp parallel for
        for (int y = 0; y < size; y++) {
            for (int x = 0; x < size; x++) {
                float free_mass = grid_cells.free_mass[y*size + x];
                float occ_mass = grid_cells.occ_mass[y*size + x];

                occ(x, y) = occ_mass + 0.5f * (1.0f - occ_mass - free_mass);
            }
        }

        grid_map_->freeGridCells(grid_cells);
        return occ;
    }

    void DOGMRos::processOdometry(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        double time_stamp = odom_msg->header.stamp.toSec();
        tf::Quaternion q;

        q.setW(odom_msg->pose.pose.orientation.w);
        q.setX(odom_msg->pose.pose.orientation.x);
        q.setY(odom_msg->pose.pose.orientation.y);
        q.setZ(odom_msg->pose.pose.orientation.z);
        auto mat = tf::Matrix3x3(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        pos_yaw = float(yaw);
    }

    cv::Mat DOGMRos::getMeasuredOccMassImage() const {
        auto grid_size = int(params_.size / params_.resolution);
        auto host_cell_data = grid_map_->getMeasurementCells();

        cv::Mat image(grid_size, grid_size, CV_8UC3);
        for (int i = 0; i < host_cell_data.size; i++) {
            cv::Vec3b color;
            int x = i % grid_size;
            int y = i / grid_size;
            color[0] = color[1] = color[2] =
                    uchar((1.0 - host_cell_data.occ_mass[i]) * 255);
            image.at<cv::Vec3b>(grid_size - x - 1, grid_size - y - 1) = color;
        }
        return image;
    }

    cv::Mat DOGMRos::getMeasuredFreeMassImage() const {
        auto grid_size = int(params_.size / params_.resolution);
        auto host_cell_data = grid_map_->getMeasurementCells();

        cv::Mat image(grid_size, grid_size, CV_8UC3);
        for (int i = 0; i < host_cell_data.size; i++) {
            cv::Vec3b color;
            int x = i % grid_size;
            int y = i / grid_size;
            color[0] = color[1] = color[2] =
                    uchar((1.0 - host_cell_data.free_mass[i]) * 255);
            image.at<cv::Vec3b>(grid_size - x - 1, grid_size - y - 1) = color;
        }
        return image;
    }

} // namespace dogm_ros
