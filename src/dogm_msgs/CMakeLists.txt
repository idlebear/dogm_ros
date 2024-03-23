cmake_minimum_required(VERSION 3.5)
project(dogm_msgs)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/DynamicOccupancyGrid.msg"
        "msg/GridCell.msg"
        "msg/GridMapInfo.msg"
        DEPENDENCIES std_msgs geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()