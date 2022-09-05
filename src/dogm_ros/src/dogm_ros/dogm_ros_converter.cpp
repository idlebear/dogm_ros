#include <fstream>
#include <sstream>
#include <iomanip>
#include <limits>

#include <chrono>

#include "dogm_ros/dogm_ros.h"
#include "dogm_ros/dogm_ros_converter.h"

#include "opencv2/highgui.hpp"

namespace dogm_ros {

    static int seq = 1000;
    static float last_x = 0, last_y = 0, last_yaw = std::numeric_limits<float>::infinity();

    void DOGMRosConverter::toDOGMMessage(const dogm::DOGM &dogm, const std::string& frame,
                                         dogm_msgs::DynamicOccupancyGrid &message,
                                         bool show_debug) {
        message.header.stamp = ros::Time::now();
        // TODO:  make the map reference frame a configurable option
        message.header.frame_id = "map";

        message.info.resolution = dogm.getResolution();
        message.info.length = float(dogm.getGridSize()) * dogm.getResolution();
        message.info.size = dogm.getGridSize();

        message.info.pose.position.x = 0;
        message.info.pose.position.y = 0;
        message.info.pose.position.z = 0.0;

        tf2::Quaternion q;
        message.info.pose.orientation.x = 0;
        message.info.pose.orientation.y = 0;
        message.info.pose.orientation.z = 0;
        message.info.pose.orientation.w = 1;

        message.data.clear();
        message.data.resize(dogm.getGridSize() * dogm.getGridSize());

        auto grid_cells = dogm.getGridCells();

        if (show_debug) {
            auto img = dogm.getNewBornOccMassImage(grid_cells);
            imshow("Born", img);
            //  img = dogm.getPredOccMassImage( grid_cells );
            //  imshow( "Predicted", img );
            //  img = dogm.getPersOccMassImage( grid_cells );
            //  imshow( "Persistent", img );
            //  img = dogm.getOccupancyImage( grid_cells );
            //  imshow( "Occupancy", img );
            img = dogm.getParticleCountImage(grid_cells);
            imshow("Particles!", img);
        }

        auto moving_cells = 0;
        for (int i = 0; i < message.data.size(); i++) {
            if (grid_cells.occ_mass[i] > 0.25) {
                auto v = sqrt(grid_cells.mean_y_vel[i] * grid_cells.mean_y_vel[i] +
                              grid_cells.mean_x_vel[i] * grid_cells.mean_x_vel[i]);
                if (v > 0.2) {
                    moving_cells++;
                }
            }
        }

        printf("Currently %d cells moving\n", moving_cells);

#if USE_SOA
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
#else
#pragma omp parallel for
        for (int i = 0; i < message.data.size(); i++) {
          message.data[i].free_mass = grid_cells[i].free_mass;
          message.data[i].occ_mass = grid_cells[i].occ_mass;

          message.data[i].mean_x_vel = grid_cells[i].mean_x_vel;
          message.data[i].mean_y_vel = grid_cells[i].mean_y_vel;
          message.data[i].var_x_vel = grid_cells[i].var_x_vel;
          message.data[i].var_y_vel = grid_cells[i].var_y_vel;
          message.data[i].covar_xy_vel = grid_cells[i].covar_xy_vel;
        }
#endif
    }

    void DOGMRosConverter::toOccupancyGridMessage(const dogm::DOGM &dogm,
                                                  const std::string& frame,
                                                  nav_msgs::OccupancyGrid &message,
                                                  bool show_debug) {

        message.header.stamp = ros::Time::now();
        // TODO:  make the map reference frame a configurable option
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

#if USE_SOA
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
#else
#pragma omp parallel for
        for (int i = 0; i < message.data.size(); i++) {
            float free_mass = grid_cells[i].free_mass;
            float occ_mass = grid_cells[i].occ_mass;

            float prob = occ_mass + 0.5f * (1.0f - occ_mass - free_mass);
            if (prob == 0.5f) {
                message.data[i] = -1;
            } else {
                message.data[i] = static_cast<char>(prob * 100.0f);
            }
        }
#endif
    }

} /* namespace dogm_ros */
