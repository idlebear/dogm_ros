#pragma once

#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include <dogm_msgs/DynamicOccupancyGrid.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace dogm_ros {

    class DOGMRosConverter {
        public:
        DOGMRosConverter() = default;

        virtual ~DOGMRosConverter() = default;

        static void toDOGMMessage(const dogm::DOGM &dogm, const std::string& frame,
                                  dogm_msgs::DynamicOccupancyGrid &message, bool show_debug = false);

        static void toOccupancyGridMessage(const dogm::DOGM &dogm, const std::string& frame,
                                           nav_msgs::OccupancyGrid &message, bool show_debug = false);
    };

} /* namespace dogm_ros */
