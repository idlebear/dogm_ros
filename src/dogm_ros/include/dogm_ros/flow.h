//
// Created by bjgilhul on 8/9/22.
//

#ifndef DOGM_ROS_FLOW_H
#define DOGM_ROS_FLOW_H

#include "Eigen/Dense"
#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>

namespace dogm_ros {
    enum FlowMethod {
        bilinear,
        nearest
    };

    enum FlowOutput {
        singular,
        cummulative
    };

    // TODO: Currently using a vector of matrices to represent a tensor.  There is a Eigen::Tensor
    //       type, but it's not clear to me (at this moment) if it's worth investing the time to implement.
    //       Look at this again when we start looking at interfacing with the PyTorch C++ api where tensors
    //       are going to be *very* useful.
    typedef std::vector<Eigen::ArrayXXf> ArrayGroup;
    typedef std::pair<Eigen::ArrayXXf,Eigen::ArrayXXf> VelocityArray;
    typedef std::vector<VelocityArray> VelocityGroup;

    class Flow {
        public:
            Flow( float scale, FlowMethod method, FlowOutput output );
            ~Flow() = default;

            Eigen::ArrayXXf step( const Eigen::ArrayXXf& occupancy, const VelocityGroup& velocity_group,
                                  const ArrayGroup& score, float dt ) const;

        private:
            float scale;
            int pad;
            FlowMethod method;
            FlowOutput output;
    };

    cv::Mat renderFlow(ArrayGroup flow_data);

}

#endif //DOGM_ROS_FLOW_H
