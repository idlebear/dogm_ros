cmake_minimum_required(VERSION 3.16)
project(dogm_ros LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_SHARED_LIBS=OFF")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/modules/")

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dogm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


find_package(orocos_kdl REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED )
find_package(Eigen3 3.4 REQUIRED)
find_package(yaml-cpp REQUIRED)


# Compile the dogm library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/dynamic-occupancy-grid-map/dogm)



## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${DOGM_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)


### Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/dogm_ros_node.cpp
        src/dogm_ros/dogm_ros_converter.cpp
        src/dogm_ros/dogm_node.cpp
        src/dogm_ros/map.cpp
        src/dogm_ros/flow.cpp
)
set_property(TARGET ${PROJECT_NAME}_node PROPERTY CUDA_ARCHITECTURES 61 75 86 )


ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        std_msgs
        geometry_msgs
        nav_msgs
        dogm_msgs
        sensor_msgs
        tf2
        tf2_ros
)

target_link_libraries( ${PROJECT_NAME}_node
        ${OpenCV_LIBS}
        ${YAML_CPP_LIBRARIES}
        dogm
        ${orocos_kdl_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME} )

ament_package()

