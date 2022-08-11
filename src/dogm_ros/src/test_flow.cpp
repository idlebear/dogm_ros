//
// Created by bjgilhul on 8/10/22.
//


#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <utility>

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

#include "dogm_ros/flow.h"


using namespace dogm_ros;

int
main( int argc, char **argv ) {

    namedWindow("Flow", cv::WINDOW_NORMAL); // Create Window

    auto rows = 40;
    auto cols = 40;

    // Build arrays to represent motion with an equal probability of heading left and down, but with slightly
    // different vectors (so we can tell them apart)
    Eigen::ArrayXXf score = Eigen::ArrayXXf::Constant(rows, cols, 0.5);
    ArrayGroup scoreGroup = { score, score };
    Eigen::ArrayXXf x_motion = Eigen::ArrayXXf::Constant(rows, cols, 2.5);
    Eigen::ArrayXXf zero_motion = Eigen::ArrayXXf::Zero(rows, cols);
    Eigen::ArrayXXf y_motion = Eigen::ArrayXXf::Constant(rows, cols, 4.5);
    VelocityGroup velocityGroup = { std::make_pair(x_motion, zero_motion), std::make_pair(zero_motion, y_motion) };

    Eigen::ArrayXXf initial_occ = Eigen::ArrayXXf::Zero(rows, cols);
    initial_occ.block( 5, 5, 10, 10 ) = 1.0;

    auto scale = 0.5f;
    Flow flow( scale, FlowMethod::nearest , FlowOutput::singular );

    ArrayGroup results = { initial_occ };
    auto steps = 5;
    for( int s = 0; s < steps; s++ ) {
        auto next = flow.step( results.back(), velocityGroup, scoreGroup, 0.5 );
        results.emplace_back( next );
    }

    auto img = renderFlow( results );
    imshow( "Flow", img );

    cv::waitKey(0);
    return(0);
}