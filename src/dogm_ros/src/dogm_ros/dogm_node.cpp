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

#include <memory>

#include "dogm_ros/dogm_node.h"
#include "dogm_ros/dogm_ros.h"

#include <dogm/dogm.h>
#include <dogm/dogm_types.h>

#include <dogm_msgs/DynamicOccupancyGrid.h>

namespace dogm_ros
{

DOGMRos::DOGMRos(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	: nh_(nh), private_nh_(private_nh), grid_map_(nullptr)
{
	std::string subscribe_laser_topic;
	private_nh_.param("subscribe/laser_topic", subscribe_laser_topic, std::string("/carla/ego_vehicle/scan"));
	std::string subscribe_odometry_topic;
    private_nh_.param("subscribe/odometry_topic", subscribe_odometry_topic, std::string("/odometry"));

    std::string publish_topic;
	private_nh_.param("publish/dogm_topic", publish_topic, std::string("/dogm/map"));

	private_nh_.param("map/size", params_.size, 50.0f);
	private_nh_.param("map/resolution", params_.resolution, 0.1f);
	private_nh_.param("particles/particle_count", params_.particle_count, 20000);
	private_nh_.param("particles/new_born_particle_count", params_.new_born_particle_count, 2000);
	private_nh_.param("particles/persistence_probability", params_.persistence_prob, 0.99f);
	private_nh_.param("particles/process_noise_position", params_.stddev_process_noise_position, 0.02f);
	private_nh_.param("particles/process_noise_velocity", params_.stddev_process_noise_velocity, 0.8f);
	private_nh_.param("particles/birth_probability", params_.birth_prob, 0.02f);
	private_nh_.param("particles/velocity_persistent", params_.stddev_velocity, 12.0f);
	private_nh_.param("particles/velocity_birth", params_.init_max_velocity, 12.0f);

	private_nh_.param("laser/fov", laser_params_.fov, 360.0f);
	private_nh_.param("laser/max_range", laser_params_.max_range, 50.0f);

	// TODO: laser resolution is currently assumed to be the same as the map
    laser_params_.resolution = params_.resolution;

    laser_conv_ = std::make_unique<dogm::LaserMeasurementGrid>(laser_params_, params_.size, params_.resolution);
	grid_map_ = std::make_unique<dogm::DOGM>( params_ );

	is_first_measurement_ = true;
	
	subscriber_laser_ = nh_.subscribe(subscribe_laser_topic, 1, &DOGMRos::processLaserScan, this);
    subscriber_odometry_ = nh_.subscribe(subscribe_odometry_topic, 1, &DOGMRos::processOdometry, this);
	publisher_ = nh_.advertise<dogm_msgs::DynamicOccupancyGrid>(publish_topic, 1);
}

void DOGMRos::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	double time_stamp = scan->header.stamp.toSec();

    auto cell_data = laser_conv_->generateGrid(scan->ranges);
    grid_map_->addMeasurementGrid(cell_data, true);

	if (!is_first_measurement_)
	{
		float dt = time_stamp - last_time_stamp_;
		grid_map_->updateGrid(dt);
 	}
	else
	{
		grid_map_->updateGrid(0.0f);
		is_first_measurement_ = false;
	}

	dogm_msgs::DynamicOccupancyGrid message;
    dogm_ros::DOGMRosConverter::toDOGMMessage(*grid_map_, message);

	publisher_.publish(message);
	
	last_time_stamp_ = time_stamp;
}

void DOGMRos::processOdometry(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    double time_stamp = odom_msg->header.stamp.toSec();

    grid_map_->updatePose( odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y );

    // TODO: Need to make sure the pose information is in sync with the the laser scan updates so
    //       the map is properly generated.  May also want to update the dogma to properly track the
    //       complete pose information

}


} // namespace dogm_ros
