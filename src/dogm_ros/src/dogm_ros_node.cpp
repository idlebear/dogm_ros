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
#include "dogm_ros/dogm_node.h"
#include <ros/ros.h>
#include <signal.h>
#include <thread>

#include "map.h"

void
shutdown_handler(int signal) {
    // Just in case we need some custom shutdown handling...
    ros::shutdown();
}

void predict( dogm_ros::DOGMRos* dogm_ptr, info_gain::Map map ) {
    // perform one iteration
    //     get the current state/update sensors
    auto occ = dogm_ptr->getOccupancyGrid();
    auto pos = dogm_ptr->getPos();

    //     make estimates
    auto estimate = map.get_flow( pos, occ, 3, 0.5 );

    int c = 0;
    for( const auto& oc : estimate ) {
        Eigen::MatrixXf oc_mat( oc.colwise().reverse() );
        cv::Mat mat(oc.rows(), oc.cols(), CV_32F );
        cv::eigen2cv(  oc_mat, mat );

        auto window_title = std::string( "Now" ) + ( c ? "+"+std::to_string(c) : "" );
        cv::imshow( window_title, mat );

        c++;
    }

    return;
}

/*
 * run_simulation
 *
 * Params:
 *
 * dogm -- the map object use use for sensor information
 * nh -- ROS node handle
 *
 */
void
run_simulation(dogm_ros::DOGMRos *dogm) {
    // TODO: Get frequency from ROS params
    // define a target refresh raite
    const int target_interval_time = 10; // hz
    auto rate = ros::Rate(target_interval_time);

    const std::string mapdir("/home/bjgilhul/workspace/labwork/phd/information-gain/ros/src/ig_car/maps" );
    const std::string mapname( "e5-0.2m");
    auto map = info_gain::Map( mapdir, mapname, true);

    // create debug windows so we can see what's going on locally
    namedWindow("Now", cv::WINDOW_NORMAL); // Create Window
    namedWindow("Now+1", cv::WINDOW_NORMAL); // Create Window
    namedWindow("Now+2", cv::WINDOW_NORMAL); // Create Window
    namedWindow("Now+3", cv::WINDOW_NORMAL); // Create Window

    cv::startWindowThread();

    while (ros::ok()) {
        dogm->publishOccupancyGrid();

        predict( dogm, map );

        rate.sleep();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "dogm_node");
    dogm_ros::DOGMRos dogm(ros::NodeHandle(), ros::NodeHandle("~"), false);
    signal(SIGINT, shutdown_handler);

    std::thread worker(run_simulation, &dogm);

    ros::spin();
    worker.join();

    return 0;
}