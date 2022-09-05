//
// Created by bjgilhul on 9/1/22.
//

#ifndef IG_CAR_MAP_H
#define IG_CAR_MAP_H

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <iostream>

#include "flow.h"

namespace info_gain {
    class Map {
        public:
        Map( const std::string& map_dir, const std::string& map_name, bool show_debug = false );
        ~Map() = default;

        Eigen::MatrixXf get_section( std::pair<float, float> center, size_t height, size_t width );

        ArrayGroup get_flow( std::pair<float, float> center, const Eigen::MatrixXf& occupancy, int steps = 10, float dt = 0.5 );


        private:
            bool show_debug;
            float  walking_speed;

            std::vector<float> origin;
            float resolution;
            bool negate;
            float occ_thresh;
            float free_thresh;

            Eigen::MatrixXf map_data;
            Eigen::MatrixXf flow_data;

            Eigen::ArrayXXf score;
            Eigen::ArrayXXf motion;
    };
}
#endif //IG_CAR_MAP_H
