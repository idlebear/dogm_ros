#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>

#include <chrono>

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

  auto grid_cells = dogm.getGridCells();

#pragma omp parallel for
  for (int i = 0; i < message.data.size(); i++) {
    message.data[i].free_mass = grid_cells.free_mass[i];
    message.data[i].occ_mass = grid_cells.occ_mass[i];

    message.data[i].mean_x_vel = grid_cells.mean_x_vel[i];
    message.data[i].mean_y_vel = grid_cells.mean_y_vel[i];
    message.data[i].var_x_vel = grid_cells.var_x_vel[i];
    message.data[i].var_y_vel = grid_cells.var_y_vel[i];
    message.data[i].covar_xy_vel = grid_cells.covar_xy_vel[i];
  }

  dogm.freeGridCells(grid_cells);

}

void DOGMRosConverter::toOccupancyGridMessage(
    const dogm::DOGM &dogm, nav_msgs::OccupancyGrid &message) {

    message.header.stamp = ros::Time::now();
    message.header.frame_id = "map";
    message.info.map_load_time = message.header.stamp;
    message.info.resolution = dogm.getResolution();
    message.info.width = dogm.getGridSize();
    message.info.height = dogm.getGridSize();

    message.info.origin.position.x = 0;
    message.info.origin.position.y = 0;
    message.info.origin.position.z = 0.0;

    message.info.origin.orientation.x = 0;
    message.info.origin.orientation.y = 0;
    message.info.origin.orientation.z = 0;
    message.info.origin.orientation.w = 1;

    message.data.clear();
    message.data.resize(dogm.getGridSize() * dogm.getGridSize());

    auto grid_cells = dogm.getGridCells();

#pragma omp parallel for
    for (int i = 0; i < message.data.size(); i++) {
        float free_mass = grid_cells.free_mass[i];
        float occ_mass = grid_cells.occ_mass[i];

        float prob = occ_mass + 0.5f * (1.0f - occ_mass - free_mass);
        if (prob == 0.5f) {
            message.data[i] = -1;
        } else {
            message.data[i] = static_cast<char>(prob * 100.0f);
        }
    }

    dogm.freeGridCells(grid_cells);
}

} /* namespace dogm_ros */
