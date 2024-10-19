//----------------------------------------------------------------------------------------------------
// File: Point.hpp
// Desc: class representing a physical elbow and wrist positions
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes

// Third-party dependencies
#include <Eigen/Dense>
#include "VirtualTunnel.hpp"
#include <unsupported/Eigen/LevenbergMarquardt>
#include <functional>

namespace gtfo{
namespace collision{

// define the object function
struct DistanceFunctor : Eigen::DenseFunctor<double> {
    Eigen::Vector3d P;  
    double s_min, s_max; 
    // input the curve and derivative of the curve
    std::function<Eigen::Vector3d(double)> r;
    std::function<Eigen::Vector3d(double)> r_prime;

    DistanceFunctor(const Eigen::Vector3d& point, double s_min_, double s_max_,
                    std::function<Eigen::Vector3d(double)> r_func,
                    std::function<Eigen::Vector3d(double)> r_prime_func)
        : Eigen::DenseFunctor<double>(1, 1), P(point), s_min(s_min_), s_max(s_max_), r(r_func), r_prime(r_prime_func) {}

    // objective function
    int operator()(const double s, double &fvec) const {

        // calculate point on curve
        Eigen::Vector3d r_s = r(s);

        // calculate the distance between the point on the curve and the given point
        fvec = (r_s - P).norm();
        return 0;
    }

    // calculate Jacobian matrix
    int df(const double s, double& fjac) const {

        // get the derivative on the point s
        Eigen::Vector3d r_prime_s = r_prime(s);

        Eigen::Vector3d r_s = r(s);

        // calculate Jacobian matrix
        fjac = ((r_s - P).normalized()).dot(r_prime_s);
        return 0;
    }

    // constrain s
    void clamp(Eigen::VectorXd& s) const {
        s[0] = std::min(s_max, std::max(s_min, s[0]));
    }
};


template<typename Scalar = double>
class Point{
public:
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

    Point(const Vector3& point_of_interest, const Vector3& point_on_curve)
        :   point_of_interest_(point_of_interest),
            point_on_curve_(point_on_curve)
    {

    }

    Vector3 PointOfInterest(void) const{
        return point_of_interest_;
    }

    Vector3 PointOnCurve(void) const{
        return point_on_curve_;
    }

    Scalar Length(void) const{
        return (point_on_curve_ - point_of_interest_).norm();
    }

    Point MinDistanceVectorTo(const Eigen::MatrixXd other) const{
        double min_dist = std::numeric_limits<double>::max();
        int index = -1;
        for (int i = 0; i < other.rows(); ++i) {
            double dist = (other.row(i) - point_of_interest_.transpose()).norm();
            if (dist < min_dist) {
                min_dist = dist;
                index = i;
            }
        }
        point_on_curve_ =  other.row(index).transpose();
        return Point(point_of_interest_, point_on_curve_);
    }

private:
    Vector3 point_of_interest_;
    Vector3 point_on_curve_;
};

}   // namespace collision
}   // namespace gtfo