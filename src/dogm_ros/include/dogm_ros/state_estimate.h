//
// Created by bjgilhul on 8/1/22.
//

#ifndef DOGM_ROS_STATE_ESTIMATE_H
#define DOGM_ROS_STATE_ESTIMATE_H

#include <utility>

#include <Eigen/Dense>

namespace dogm_ros {


    class StateEstimate {
        // Given position measurements (x,y) in cartesian coordinates, calculate the velocity estimate
        public:
            StateEstimate(Eigen::Matrix4d Qi, Eigen::Matrix2d Ri ) :
                X_est(Eigen::Vector4d::Zero()), S_est( Eigen::Matrix4d::Zero() ), Q(std::move(Qi)), R(std::move(Ri)),
                H( 2,4 ), Z_prev( {0, 0} ), measurement_count(0)  {
                H << 1, 0, 0, 0,
                     0, 1, 0, 0;
            }
            ~StateEstimate() = default;

            std::pair<Eigen::Vector4d, Eigen::Matrix4d> step( const Eigen::Vector2d& Z, double dt );

        private:
            // internal state and covariance
            Eigen::Vector4d X_est;
            Eigen::Matrix4d S_est;

            // uncertainty
            Eigen::Matrix4d Q;
            Eigen::Matrix2d R;

            // state -> measurement
            Eigen::MatrixXd H;

        // previous measurement (necessary to calculate
            Eigen::Vector2d Z_prev;
            int measurement_count;
    };



}


#endif //DOGM_ROS_STATE_ESTIMATE_H
