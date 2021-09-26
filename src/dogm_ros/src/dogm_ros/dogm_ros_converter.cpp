#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>

#include "dogm_ros/dogm_ros_converter.h"

namespace dogm_ros {

  static int seq = 1000;
  static float last_x = 0, last_y = 0, last_yaw = std::numeric_limits<float>::infinity();

void DOGMRosConverter::toDOGMMessage(const dogm::DOGM &dogm,
                                     dogm_msgs::DynamicOccupancyGrid &message) {
  message.header.stamp = ros::Time::now();
  // TODO:  should be publishing the map in the car's reference frame
  message.header.frame_id = "map";

  message.info.resolution = dogm.getResolution();
  message.info.length = float(dogm.getGridSize()) * dogm.getResolution();
  message.info.size = dogm.getGridSize();

  message.info.pose.position.x = 0;
  message.info.pose.position.y = 0;
  //  message.info.pose.position.x = dogm.getPositionX();
  //  message.info.pose.position.y = dogm.getPositionY();
  message.info.pose.position.z = 0.0;

  tf2::Quaternion q;
  //  q.setRPY( 0, 0, dogm.getYaw() );
  //  message.info.pose.orientation.x = q.x();
  //  message.info.pose.orientation.y = q.y();
  //  message.info.pose.orientation.z = q.z();
  //  message.info.pose.orientation.w = q.w();
  message.info.pose.orientation.x = 0;
  message.info.pose.orientation.y = 0;
  message.info.pose.orientation.z = 0;
  message.info.pose.orientation.w = 1;

  message.data.clear();
  message.data.resize(dogm.getGridSize() * dogm.getGridSize());

  const auto grid_cells = dogm.getGridCells();

#pragma omp parallel for
  for (int i = 0; i < message.data.size(); i++) {
    auto cell = grid_cells[i];

    message.data[i].free_mass = cell.free_mass;
    message.data[i].occ_mass = cell.occ_mass;

    message.data[i].mean_x_vel = cell.mean_x_vel;
    message.data[i].mean_y_vel = cell.mean_y_vel;
    message.data[i].var_x_vel = cell.var_x_vel;
    message.data[i].var_y_vel = cell.var_y_vel;
    message.data[i].covar_xy_vel = cell.covar_xy_vel;
  }
}

void DOGMRosConverter::toOccupancyGridMessage(
    const dogm::DOGM &dogm, nav_msgs::OccupancyGrid &message) {

  auto x = dogm.getPositionX();
  auto y = dogm.getPositionY();
  auto yaw = dogm.getYaw();

  message.header.stamp = ros::Time::now();
  message.header.frame_id = "map";
  message.info.map_load_time = message.header.stamp;
  message.info.resolution = dogm.getResolution();
  message.info.width = dogm.getGridSize();
  message.info.height = dogm.getGridSize();

   message.info.origin.position.x = 0;
   message.info.origin.position.y = 0;
  //  message.info.origin.position.x = x;
  //  message.info.origin.position.y = y;
  message.info.origin.position.z = 0.0;

  tf2::Quaternion q;
  //  q.setRPY( 0, 0, yaw );
  //  message.info.origin.orientation.x = q.x();
  //  message.info.origin.orientation.y = q.y();
  //  message.info.origin.orientation.z = q.z();
  //  message.info.origin.orientation.w = q.w();
  message.info.origin.orientation.x = 0;
  message.info.origin.orientation.y = 0;
  message.info.origin.orientation.z = 0;
  message.info.origin.orientation.w = 1;


  message.data.clear();
  message.data.resize(dogm.getGridSize() * dogm.getGridSize());

  const auto grid_cells = dogm.getGridCells();

#pragma omp parallel for
  for (int i = 0; i < message.data.size(); i++) {
    auto cell = grid_cells[i];
    float free_mass = cell.free_mass;
    float occ_mass = cell.occ_mass;

    float prob = occ_mass + 0.5f * (1.0f - occ_mass - free_mass);

    if (prob == 0.5f) {
      message.data[i] = -1;
    } else {
      message.data[i] = static_cast<char>(prob * 100.0f);
    }
  }
}

} /* namespace dogm_ros */
