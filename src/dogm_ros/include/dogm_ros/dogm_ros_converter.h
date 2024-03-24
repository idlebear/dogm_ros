#pragma once

#include <dogm/dogm.h>
#include <dogm/dogm_types.h>
#include <dogm_msgs/msg/dynamic_occupancy_grid.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dogm_ros {

class DOGMRosConverter {
public:
    DOGMRosConverter() = default;

    virtual ~DOGMRosConverter() = default;

    static void toDOGMMessage(rclcpp::Time time, const dogm::DOGM& dogm, const std::string& frame,
                              dogm_msgs::msg::DynamicOccupancyGrid& message, bool show_debug = false);

    static void toOccupancyGridMessage(rclcpp::Time time, const dogm::DOGM& dogm, const std::string& frame,
                                       nav_msgs::msg::OccupancyGrid& message, bool show_debug = false);
};

} /* namespace dogm_ros */
