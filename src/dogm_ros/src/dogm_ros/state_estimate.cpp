//
// Created by bjgilhul on 8/1/22.
//

#include "dogm_ros/state_estimate.h"
#include "Eigen/Dense"

namespace dogm_ros {
std::pair<Eigen::Vector4d, Eigen::Matrix4d> StateEstimate::step(const Eigen::Vector2d& Z, double dt)
{

    // estimate the next state based on the measurements
    if (!measurement_count) {
        X_est << Z(0), Z(1), 0, 0;
        S_est = Eigen::Matrix4d::Zero();
        S_est.topLeftCorner(2, 2) = R;
    } else if (measurement_count == 1) {
        X_est << Z(0), Z(1), (Z(0) - Z_prev(0)) / dt, (Z(1) - Z_prev(1)) / dt;
        S_est = Eigen::Matrix4d::Zero();
        S_est.topLeftCorner(2, 2) = R;
        S_est.bottomRightCorner(2, 2) = Eigen::Matrix2d({{1000, 0}, {0, 1000}});
    } else {
        // move according to current estimates and update predictions
        Eigen::Matrix4d A{{1, 0, dt, 0}, {0, 1, 0, dt}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        auto X_pred = A * X_est;
        auto S_pred = A * S_est * A.transpose() + Q;  // Plus some estimate of noise

        // calculate new kalman value
        auto K = S_pred * H.transpose() * (H * S_pred * H.transpose() + R).inverse();

        // update estimate based on surprise value in measurements
        X_est = X_pred + K * (Z - H * X_pred);
        S_est = S_pred - K * H * S_pred;
    }

    // store the previous measurement for next time.
    Z_prev = Z;
    measurement_count++;

    return std::make_pair(X_est, S_est);
}

}  // namespace dogm_ros
