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
#pragma once

#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include <dogm/mapping/laser_to_meas_grid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

namespace dogm_ros {

class DOGMRos {
public:
  DOGMRos(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);

  virtual ~DOGMRos() = default;

  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan);
  void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr &scan);
  void processOdometry(const nav_msgs::Odometry::ConstPtr &odom_msg);

protected:
  void processSensorScanData(float time_stamp, const std::vector<float> &data);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber subscriber_laser_;
  ros::Subscriber subscriber_odometry_;
  ros::Publisher publisher_dogm_;
  ros::Publisher publisher_occ_;

  dogm::DOGM::Params params_;
  dogm::LaserMeasurementGrid::Params laser_params_;

  float last_time_stamp_;
  bool is_first_measurement_;

  std::unique_ptr<dogm::LaserMeasurementGrid> laser_conv_;
  std::unique_ptr<dogm::DOGM> grid_map_;

  float lidar_increment;
  float lidar_min_height;
  float lidar_max_height;

  float pos_x;
  float pos_y;
  float yaw;
};

} // namespace dogm_ros
