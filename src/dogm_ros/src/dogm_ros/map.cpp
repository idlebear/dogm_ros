//
// Created by bjgilhul on 9/1/22.
//

#include "map.h"
#include <opencv2/highgui.hpp>

namespace info_gain {

    Map::Map( const std::string& map_dir, const std::string& map_name, bool show_debug )
        : show_debug(show_debug), walking_speed( 2.0 ) {
        auto config_file_name = map_dir + "/" + map_name + ".yaml";
        YAML::Node config = YAML::LoadFile(config_file_name);
        // Sample map configuration file
        //
        //    image: e5-0.1m.pgm
        //    resolution: 0.200000
        //    origin: [15.341689, -78.008743, 0.000000]
        //    negate: 0
        //    occupied_thresh: 0.65
        //    free_thresh: 0.196
        //
        const std::string map_data_name = config["image"].as<std::string>();
        resolution = config["resolution"].as<double>();
        origin = config["origin"].as<std::vector<float>>();
        negate = bool(config["negate"].as<int>());
        occ_thresh = config["occupied_thresh"].as<float>();
        free_thresh = config["free_thresh"].as<float>();

        auto map_data_path = map_dir + "/" + map_data_name;
        cv::Mat map_img_uchar = cv::imread( map_data_path, cv::IMREAD_UNCHANGED);
        cv::Mat map_img;
        map_img_uchar.convertTo( map_img, CV_32F );
        map_data = Eigen::MatrixXf( map_img.rows, map_img.cols );
        cv::cv2eigen( map_img, map_data );

        auto flow_data_path = map_dir + "/" + map_name + ".flow";
        cv::Mat flow_img_uchar = cv::imread( flow_data_path, cv::IMREAD_UNCHANGED);
        double minv, maxv;
        cv::minMaxLoc( flow_img_uchar, &minv, &maxv );
        cv::Mat flow_img;
        flow_img_uchar.convertTo( flow_img, CV_32F, 1.0/(maxv - minv), -minv/(maxv - minv) );
        flow_data = Eigen::MatrixXf( flow_img.rows, flow_img.cols );
        cv::cv2eigen( flow_img, flow_data );
        cv::minMaxLoc( flow_img, &minv, &maxv );

        // Build arrays to represent motion with an equal probability of moving in any of the cardinal directions)
        auto mask = (flow_data.array() > free_thresh).cast<float>();
        score = Eigen::ArrayXXf::Constant(flow_img.rows, flow_img.cols, 0.25);
        score *= mask;

        motion = Eigen::ArrayXXf::Constant(flow_img.rows, flow_img.cols, walking_speed);
        motion *= mask;

        if( show_debug ) {
            namedWindow("Score", cv::WINDOW_NORMAL); // Create Window
            namedWindow("Motion", cv::WINDOW_NORMAL); // Create Window
        }
    }

    Eigen::MatrixXf Map::get_section( std::pair<float, float> center, size_t height, size_t width ) {

        auto cx = int( ( center.first - origin[0] ) / resolution );
        auto cy = int( ( center.second - origin[1] ) / resolution );

        auto min_y = std::max( 0, int( cx - height / 2 ) );
        auto min_x = std::max( 0, int( cy - width / 2 ) );
        auto max_y = std::min( int(height), int(cx - height / 2));
        auto max_x = std::min( int(width), int(cx - width / 2));

        Eigen::MatrixXf res = Eigen::MatrixXf::Ones( height, width );
        res.block( min_y, min_x,  max_y - min_y, max_x - min_x ) = map_data.block( min_x, min_y,  max_y - min_y, max_x - min_x );
        return res;
    }

    ArrayGroup Map::get_flow( std::pair<float, float> center, const Eigen::MatrixXf& occupancy, int steps, float dt ) {

        auto height = occupancy.rows();
        auto width = occupancy.cols();

        auto cx = int( ( center.first - origin[0] ) / resolution );
        auto cy = int( ( center.second - origin[1] ) / resolution );

        if( cx < 0 || cx > map_data.cols() || cy < 0 || cy > map_data.rows() ) {
            return { occupancy };
        }

        auto min_y = std::max( 0, int( cy - height / 2 ) );
        auto max_y = std::min( int(map_data.rows()), int(cy + height / 2));
        auto block_start_y = height / 2 - (cy - min_y);
        auto block_len_y = max_y - min_y;

        auto min_x = std::max( 0, int( cx - width / 2 ) );
        auto max_x = std::min( int(map_data.cols()), int(cx + width / 2));
        auto block_start_x = width / 2 - (cx - min_x);
        auto block_len_x = max_x - min_x;

        Eigen::ArrayXXf score_block = Eigen::ArrayXXf::Zero( height, width );
        score_block.block( block_start_y, block_start_x,  block_len_y, block_len_x ) = score.block( min_y, min_x,  block_len_y, block_len_x );
        ArrayGroup score_group = { score_block, score_block, score_block, score_block };

        auto zero_block = Eigen::ArrayXXf::Zero( height, width );
        Eigen::ArrayXXf motion_block = Eigen::ArrayXXf::Zero( height, width );
        motion_block.block( block_start_y, block_start_x,  max_y - min_y, max_x - min_x ) = motion.block( min_y, min_x,  max_y - min_y, max_x - min_x );
        VelocityGroup velocity_group = {
                std::make_pair(motion_block, zero_block),
                std::make_pair(zero_block, motion_block),
                std::make_pair(-motion_block, zero_block),
                std::make_pair(zero_block, -motion_block)
        };

        if( show_debug ) {
            Eigen::MatrixXf em = score_block.colwise().reverse();
            cv::Mat img( height, width, CV_32F );
            cv::eigen2cv( em, img );
            cv::imshow( "Score", img );
            em = (motion_block.colwise().reverse()) * 256.0 / 2.0;
            cv::eigen2cv( em, img );
            cv::imshow( "Motion", img );
        }

        Flow flow( resolution, FlowMethod::bilinear , FlowOutput::singular );
        ArrayGroup results = { occupancy };
        for( int s = 0; s < steps; s++ ) {
            auto next = flow.step( results.back(), velocity_group, score_group, dt );
            results.emplace_back( next );
        }

        return results;
    }

}
