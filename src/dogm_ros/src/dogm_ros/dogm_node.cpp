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
#include <limits>

#include <dogm_msgs/DynamicOccupancyGrid.h>

#include "dogm_ros/dogm_node.h"
#include "dogm_ros/dogm_ros.h"



namespace dogm_ros
{

DOGMRos::DOGMRos(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	: nh_(nh), private_nh_(private_nh), grid_map_(nullptr)
{
	std::string subscribe_laser_topic;
    private_nh_.param("subscribe/laser_topic", subscribe_laser_topic, std::string("/carla/ego_vehicle/lidar"));
//    private_nh_.param("subscribe/laser_topic", subscribe_laser_topic, std::string("/carla/ego_vehicle/scan"));
	std::string subscribe_odometry_topic;
    private_nh_.param("subscribe/odometry_topic", subscribe_odometry_topic, std::string("/carla/ego_vehicle/odometry"));

    std::string publish_dogm_topic;
	private_nh_.param("publish/dogm_topic", publish_dogm_topic, std::string("/dogm/map"));
    std::string publish_occ_topic;
    private_nh_.param("publish/occ_topic", publish_occ_topic, std::string("/dogm/occ"));

    private_nh_.param("lidar_increment", lidar_increment, 0.0087f);
    private_nh_.param("lidar_min_height", lidar_min_height, -1.5f);
    private_nh_.param("lidar_max_height", lidar_max_height, 2.0f);

    private_nh_.param("map/size", params_.size, 100.0f);
	private_nh_.param("map/resolution", params_.resolution, 0.1f);
	private_nh_.param("particles/particle_count", params_.particle_count, 20000);
	private_nh_.param("particles/new_born_particle_count", params_.new_born_particle_count, 2000);
	private_nh_.param("particles/persistence_probability", params_.persistence_prob, 0.99f);
	private_nh_.param("particles/process_noise_position", params_.stddev_process_noise_position, 0.02f);
	private_nh_.param("particles/process_noise_velocity", params_.stddev_process_noise_velocity, 0.8f);
	private_nh_.param("particles/birth_probability", params_.birth_prob, 0.02f);
	private_nh_.param("particles/velocity_persistent", params_.stddev_velocity, 12.0f);
	private_nh_.param("particles/velocity_birth", params_.init_max_velocity, 12.0f);

	// private_nh_.param("laser/fov", laser_params_.fov, 360.0f);
    private_nh_.param("laser/fov", laser_params_.fov, 360.0f);
	private_nh_.param("laser/max_range", laser_params_.max_range, 50.0f);

	// TODO: laser resolution is currently assumed to be the same as the map
    laser_params_.resolution = params_.resolution;
    laser_conv_ = std::make_unique<dogm::LaserMeasurementGrid>(laser_params_, params_.size, params_.resolution);
	grid_map_ = std::make_unique<dogm::DOGM>( params_ );

	is_first_measurement_ = true;
	
	subscriber_laser_ = nh_.subscribe(subscribe_laser_topic, 1, &DOGMRos::processPointCloud, this);
//    subscriber_laser_ = nh_.subscribe(subscribe_laser_topic, 1, &DOGMRos::processLaserScan, this);
    subscriber_odometry_ = nh_.subscribe(subscribe_odometry_topic, 1, &DOGMRos::processOdometry, this);
    publisher_dogm_ = nh_.advertise<dogm_msgs::DynamicOccupancyGrid>(publish_dogm_topic, 1);
    publisher_occ_ = nh_.advertise<nav_msgs::OccupancyGrid>(publish_occ_topic, 1);
}


void DOGMRos::processPointCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    double time_stamp = cloud->header.stamp.toSec();

    // TODO -- eventually make this a tad more general...
    const int POINT_SIZE = 4;
    assert( cloud->is_bigendian == false && cloud->point_step == 4 * POINT_SIZE );

    auto points = int( cloud->height * cloud->width );

    auto x_offset = 0;
    auto y_offset = 0;
    auto z_offset = 0;
    for( const auto& entry : cloud->fields ) {
        if( entry.name[0] == 'x' ) {
            x_offset = entry.offset;
        } else if( entry.name[0] == 'y' ) {
            y_offset = entry.offset;
        } else if( entry.name[0] == 'z' ) {
            z_offset = entry.offset;
        }
    }

    // TODO: Lidar FOV is hard-coded to two pi for the time being
    auto ray_data = std::vector<float>( int(2 * M_PI / lidar_increment),  std::numeric_limits<float>::infinity() );

    const unsigned char *data_ptr = cloud->data.data();
    for( int i = 0; i < points; i++ ) {
        auto x_idx = i * cloud->point_step + x_offset;
        auto y_idx = i * cloud->point_step + y_offset;
        auto z_idx = i * cloud->point_step + z_offset;

        auto x = *( (float *)(data_ptr + x_idx) );
        auto y = *( (float *)(data_ptr + y_idx) );
        auto z = *( (float *)(data_ptr + z_idx) );

        // filter points too high or too low
        if( z < lidar_min_height || z > lidar_max_height ) {
            continue;
        }

        auto angle = atan2( y, x ) + M_PI;
        auto range = sqrt( x*x + y*y + z*z );
        auto bucket = floor( angle / lidar_increment );

        if( range < ray_data[bucket] ) {
            ray_data[bucket] = range;
        }
    }

    processSensorScanData(time_stamp, ray_data);
}

void DOGMRos::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    double time_stamp = scan->header.stamp.toSec();

    processSensorScanData(time_stamp, scan->ranges);
}

void DOGMRos::processSensorScanData( double time_stamp, const std::vector<float>& data ) {
    auto cell_data = laser_conv_->generateGrid(data);
    grid_map_->addMeasurementGrid(cell_data, true);

	if (!is_first_measurement_)
	{
		float dt = float(time_stamp) - last_time_stamp_;
		grid_map_->updateGrid(dt);
 	}
	else
	{
		grid_map_->updateGrid(0.0f);
		is_first_measurement_ = false;
	}

//    dogm_msgs::DynamicOccupancyGrid message;
//    dogm_ros::DOGMRosConverter::toDOGMMessage(*grid_map_, message);
//    publisher_dogm_.publish(message);
    nav_msgs::OccupancyGrid message;
    dogm_ros::DOGMRosConverter::toOccupancyGridMessage(*grid_map_, message);
    publisher_occ_.publish(message);

	last_time_stamp_ = time_stamp;
}

void DOGMRos::processOdometry(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    double time_stamp = odom_msg->header.stamp.toSec();
    tf::Quaternion q;

    q.setW( odom_msg->pose.pose.orientation.w);
    q.setX( odom_msg->pose.pose.orientation.x);
    q.setY( odom_msg->pose.pose.orientation.y);
    q.setZ( odom_msg->pose.pose.orientation.z);
    auto mat = tf::Matrix3x3(q);
    double roll, pitch, yaw;
    mat.getRPY( roll, pitch, yaw );

    grid_map_->updatePose( odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, yaw );

    // TODO: Need to make sure the pose information is in sync with the the laser scan updates so
    //       the map is properly generated.  May also want to update the dogma to properly track the
    //       complete pose information

}


} // namespace dogm_ros
