//
// Created by bjgilhul on 8/9/22.
//

#include "flow.h"

namespace info_gain {

        //    Initialize the FlowGrid with a starting probability and motion vectors.  The
        //    motion and score parameters are applied over T time steps, with K instances for
        //    each one.  The score parameter allocates the strength of each vector set.
        //
        //    Parameters:
        //    -----------
        //    scale: Scale of each element in the grid, used to determine motion distances
        //
        //    score: Proportion of each motion vector to apply over the K vectors. The score can be
        //           constant over all time intervals or specified for each one.  Score has the
        //           shape:
        //
        //          (T, K, 1, x_dim, y_dim)  --or-- (T, 1, 1, x_dim, y_dim)
        //
        //    motion: X and Y motion vector grids of the shape:
        //
        //          (T, K, 2, x_dim, y_dim)
        //
        Flow::Flow( float scale, FlowMethod method, FlowOutput output ) : scale( scale ), pad(1), method(method), output(output) {}

        inline auto clip(const Eigen::ArrayXXf& A, float low, float high ) {
            return A.max(low).min(high);
        }

        inline auto padArray(const Eigen::ArrayXXf& A, int pad ) {
            if( !pad ) {
                return A;
            }
            Eigen::ArrayXXf B = Eigen::ArrayXXf::Zero( A.rows() + pad * 2, A.cols() + pad * 2 );
            B.block( pad, pad, A.rows(), A.cols() ) = A;
            return B;
        }

        inline auto arrayMultiplyByIndex( const Eigen::ArrayXXf& A, const Eigen::ArrayXXf& x_index, const Eigen::ArrayXXf& y_index,
                                          const Eigen::ArrayXXf& multiplier ) {
            assert( A.rows() == x_index.rows() && A.rows() == y_index.rows() && A.rows() == multiplier.rows() );
            assert( A.cols() == x_index.cols() && A.cols() == y_index.cols() && A.cols() == multiplier.cols() );
            Eigen::ArrayXXf res = A;
            for( int x = 0; x < A.cols(); x++ ) {
                for( int y = 0; y < A.rows(); y++ ) {
                    auto xr = int(x_index(x,y));
                    auto yr = int(y_index(x,y));
                    res(xr,yr) *= multiplier(x,y);
                }
            }
            return res;
        }

        typedef Eigen::ArrayXXf (*MapFn)( const Eigen::ArrayXXf&, const Eigen::ArrayXXf& );
        typedef Eigen::ArrayXXf (*OutputFn)( const Eigen::ArrayXXf&, const Eigen::ArrayXXf& );

        //
        //    flow all of the elements of the grid according to their velocities
        //
        //    The probability of a cell being flowed into is 1 - PI[ 1 - F(i->j)] where
        //            F(i->j) is the probability of cell i moving into j
        //
        // TODO: There's a lot of memory allocation and copying going on here -- or at least it appears that
        //       way at first glance.  Not sure how much is necessary and/or can be improved.  Revisit this
        //       implementation with a profiler at some point...
        Eigen::ArrayXXf Flow::step( const Eigen::ArrayXXf& occupancy, const VelocityGroup& velocity_group,
                                const ArrayGroup& score, float dt ) const {

            auto rows = occupancy.rows() + pad * 2;
            auto cols = occupancy.cols() + pad * 2;

            // pad the inputs
            Eigen::ArrayXXf padded_occupancy = padArray( occupancy, pad );
            VelocityGroup padded_velocity;
            for( auto & vel : velocity_group ) {
                padded_velocity.emplace_back( std::make_pair(padArray(vel.first, pad), padArray(vel.second, pad)));
            }
            ArrayGroup padded_score;
            for( auto & s : score ) {
                padded_score.emplace_back( padArray(s, pad) );
            }

            MapFn map_fn;
            if( method == FlowMethod::nearest ) {
                map_fn = []( const Eigen::ArrayXXf&  mo, const Eigen::ArrayXXf&  int_mo ) {
                    Eigen::ArrayXXf res = ((int_mo - mo) > 0.5f).cast<float>();
                    return res;
                };
            } else { // bilinear
                map_fn = []( const Eigen::ArrayXXf&  mo, const Eigen::ArrayXXf&  int_mo ) {
                    Eigen::ArrayXXf res =  int_mo - mo;
                    return res;
                };
            }

            // Taking the max of the current and the previous can lead to a more obvious flow, but
            // it also means that the occupancy stretches from the start at the last time step
            // instead of being an expanding wave.

            // Being that we can't be sure where in that cloud the car really is, maybe that's
            // the right way to do it?  It's also much noisier, resulting in a larger cloud
            // of uncertainty
            OutputFn output_fn;
            if( output == FlowOutput::singular ) {
                output_fn = []( const Eigen::ArrayXXf& prev, const Eigen::ArrayXXf& cur ) {
                    Eigen::ArrayXXf res = prev.cwiseMax(cur);
                    return res;
                };
            } else { // cummulative
                output_fn = []( const Eigen::ArrayXXf& prev, const Eigen::ArrayXXf& cur ) { return cur; };
            }

            Eigen::ArrayXXf next_prob = Eigen::ArrayXXf::Constant( rows, cols, 1 );
            for( int k = 0; k < padded_score.size(); k++ ) {
                // TODO: Reversed X and Y here in flow because the data from Carla has the X, Y coordinates flipped
                Eigen::ArrayXXf xx = Eigen::VectorXf::LinSpaced(rows, 0, float(rows-1) ).replicate(1,cols);
                Eigen::ArrayXXf yy = Eigen::RowVectorXf::LinSpaced(cols, 0, float(cols-1) ).replicate(rows,1);

                Eigen::ArrayXXf nx = xx + padded_velocity[k].first * dt / scale;
                Eigen::ArrayXXf ny = yy + padded_velocity[k].second * dt / scale;

                Eigen::ArrayXXf x1 = nx.floor();
                Eigen::ArrayXXf y1 = ny.floor();
                Eigen::ArrayXXf x2 = x1 + 1;
                Eigen::ArrayXXf y2 = y1 + 1;

                auto x1prop = map_fn(nx, x2);
                auto y1prop = map_fn(ny, y2);
                auto x2prop = 1.0f - x1prop;
                auto y2prop = 1.0f - y1prop;

                // after proportioning the blame, clip the indices so they
                // still fit in the grid -- this means our zero rows are probably
                // garbage
                x1 = clip(x1, 0l, cols-1).floor();
                x2 = clip(x2, 0l, cols-1).floor();
                y1 = clip(y1, 0l, rows-1).floor();
                y2 = clip(y2, 0l, rows-1).floor();

                Eigen::ArrayXXf prob = padded_occupancy * padded_score[k];
                Eigen::ArrayXXf k_prob = Eigen::ArrayXXf::Constant( rows, cols, 1 );

                k_prob = arrayMultiplyByIndex( k_prob, x1, y1, (1 - prob * x1prop * y1prop));
                k_prob = arrayMultiplyByIndex( k_prob, x1, y2, (1 - prob * x1prop * y2prop));
                k_prob = arrayMultiplyByIndex( k_prob, x2, y1, (1 - prob * x2prop * y1prop));
                k_prob = arrayMultiplyByIndex( k_prob, x2, y2, (1 - prob * x2prop * y2prop));

                // add the cummulative prob for this K to the prob of not flowing
                next_prob *= k_prob;
            }

            // reverse the probabilistic sense to get prob of flow for all 'j' cells and clip to
            // ensure we remain in bounds
            Eigen::ArrayXXf res = clip(1 - next_prob, 0.0f, 1.0f).block( pad, pad, occupancy.rows(), occupancy.cols());
            return res;
        }



    cv::Mat renderFlow(ArrayGroup flow_data) {
        int rows = int(flow_data[0].rows());
        int cols = int(flow_data[0].cols());

        cv::Mat image(rows, cols * int(flow_data.size()), CV_8UC3);

        float min_min_val = std::numeric_limits<float>::infinity();
        float max_max_val = -std::numeric_limits<float>::infinity();
        for (auto & fd : flow_data) {
            auto min_val = fd.minCoeff();
            auto max_val = fd.maxCoeff();
            if( min_val < min_min_val ) {
                min_min_val = min_val;
            }
            if( max_val > max_max_val ) {
                max_max_val = max_val;
            }
        }

        auto scale = max_max_val - min_min_val;
        for( int i = 0; i < flow_data.size(); i++ ) {
            for( int y = 0; y < rows; y++ ) {
                for( int x = 0; x < cols; x++ ) {
                    cv::Vec3b color;
                    color[0] = color[1] = color[2] = uchar((flow_data[i](x,y) - min_min_val)*255/scale);
                    image.at<cv::Vec3b>(rows - y - 1, i * cols + x) = color;
                }
            }
        }
        return image;
    }


}
