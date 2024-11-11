//----------------------------------------------------------------------------------------------------
// File: VirtualTunnel.hpp
// Desc: class representing a virtual tunnel
//----------------------------------------------------------------------------------------------------
// Standard libraries includes
#include <type_traits>
#include <vector>
#include <memory>
#include <functional>

// Third-party dependencies
#include <Eigen/Dense>

// Project-specific
#include "EntityPointTunnel.hpp"
#include "../Core/Utils/ClosestVector.hpp"
#include "../Core/Models/DynamicsBase.hpp"

namespace gtfo{
namespace collision{

template< typename TunnelParameters, typename Scalar = double>
class VirtualTunnel : public EntityPointTunnel<Scalar>{
public:

    using Vector3 = typename EntityPointTunnel<Scalar>::Vector3;
    
    VirtualTunnel(const TunnelParameters &tunnel_parameters)  // the vertices are actually dots along the reference trajectory
    :   EntityPointTunnel<Scalar>(true),
        number_of_vertices_(tunnel_parameters.num_of_points)
    {
        // generate trajectory in joint space first
        Eigen::Matrix<Scalar, 1, Eigen::Dynamic> pct = Eigen::Matrix<Scalar, -1, 1>::LinSpaced(tunnel_parameters.num_of_points, 0.0, 1.0);
        Eigen::Matrix<Scalar, 4, Eigen::Dynamic> pct_replicated = pct.replicate(4, 1);
        Eigen::Matrix<Scalar, 4, Eigen::Dynamic> theta;
        Eigen::Matrix<Scalar, 4, Eigen::Dynamic> start_matrix = tunnel_parameters.start_configuration.replicate(1, tunnel_parameters.num_of_points);
        Eigen::Matrix<Scalar, 4, Eigen::Dynamic> end_matrix = tunnel_parameters.end_configuration.replicate(1, tunnel_parameters.num_of_points);

        Eigen::Matrix<Scalar, 4, Eigen::Dynamic> pct3_replicated = pct_replicated.array() * pct_replicated.array() * pct_replicated.array(); // pct^3
        Eigen::Matrix<Scalar, 4, Eigen::Dynamic> pct4_replicated = pct3_replicated.array() * pct_replicated.array(); // pct^4
        Eigen::Matrix<Scalar, 4, Eigen::Dynamic> pct5_replicated = pct4_replicated.array() * pct_replicated.array();

        theta = start_matrix.array() + 
        (start_matrix.array() - end_matrix.array()) * 
        (15 * pct4_replicated.array() - 6 * pct5_replicated.array() - 10 * pct3_replicated.array());

        Eigen::Matrix<Scalar, 1, -1> t1 = theta.row(0);
        Eigen::Matrix<Scalar, 1, -1> t2 = theta.row(1);
        Eigen::Matrix<Scalar, 1, -1> t3 = theta.row(2);
        Eigen::Matrix<Scalar, 1, -1> t4 = theta.row(3);

        // convert to task space
        Eigen::Matrix<Scalar, 1, -1> elbow_x = -tunnel_parameters.lu * (t2.array().cos() * t1.array().sin());
        Eigen::Matrix<Scalar, 1, -1> elbow_y = tunnel_parameters.lu * t2.array().sin();
        Eigen::Matrix<Scalar, 1, -1> elbow_z = -tunnel_parameters.lu * (t1.array().cos() * t2.array().cos());
        
        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> elbow(3, tunnel_parameters.num_of_points);
        elbow.row(0) = elbow_x;
        elbow.row(1) = elbow_y;
        elbow.row(2) = elbow_z;
    
        for (int i = 0; i < elbow.cols(); ++i) {
            elbow_position_.push_back(elbow.col(i)); 
        }

        Eigen::Matrix<Scalar, 1, -1> wrist_x = tunnel_parameters.lf * (t4.array().sin() * (t1.array().cos() * t3.array().sin() + t3.array().cos() * t1.array().sin() * t2.array().sin()) - t2.array().cos() * t4.array().cos() * t1.array().sin()) - tunnel_parameters.lu * t2.array().cos() * t1.array().sin();
        Eigen::Matrix<Scalar, 1, -1> wrist_y = tunnel_parameters.lf * (t4.array().cos() * t2.array().sin() + t2.array().cos() * t3.array().cos() * t4.array().sin()) + tunnel_parameters.lu * t2.array().sin();
        Eigen::Matrix<Scalar, 1, -1> wrist_z = -tunnel_parameters.lf * (t4.array().sin() * (t1.array().sin() * t3.array().sin() - t1.array().cos() * t3.array().cos() * t2.array().sin()) + t1.array().cos() * t2.array().cos() * t4.array().cos()) - tunnel_parameters.lu * t1.array().cos() * t2.array().cos();

        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> wrist(3, tunnel_parameters.num_of_points);
        wrist.row(0) = wrist_x;
        wrist.row(1) = wrist_y;
        wrist.row(2) = wrist_z;

    
        for (int i = 0; i < wrist.cols(); ++i) {
            wrist_position_.push_back(wrist.col(i)); 
        }

        Vector3 dir_nor;
        if (std::abs((elbow_position_[0].normalized()).dot((wrist_position_[0] - elbow_position_[0]).normalized())) < 0.98) {
            dir_nor = ((elbow_position_[0].normalized()).cross((wrist_position_[0] - elbow_position_[0]).normalized())).normalized();
        } else {
            dir_nor = ((elbow_position_.back().normalized()).cross((wrist_position_.back() - elbow_position_.back())).normalized());
        }

        UpdateTunnelVertices(elbow_position_, wrist_position_, dir_nor);

    }

    void UpdateVirtualState() override {
    }

    void UpdateVertices(const std::vector<Vector3>& vertices) override{}

    void UpdateTunnelVertices(const std::vector<Vector3>& vertices_elbow, const std::vector<Vector3>& vertices_wrist, const Vector3& dir_nor) {
        assert(vertices_elbow.size() == number_of_vertices_);
        assert(vertices_elbow.size() >= 1);
        assert(vertices_wrist.size() == number_of_vertices_);
        assert(vertices_wrist.size() >= 1);
        EntityPointTunnel<Scalar>::vertices_elbow_ = vertices_elbow;
        EntityPointTunnel<Scalar>::vertices_wrist_ = vertices_wrist;
        EntityPointTunnel<Scalar>::dir_nor_ = dir_nor;
    }

    std::vector<Vector3> GetElbowTrajectory(void) const{
        return elbow_position_;
    }

    std::vector<Vector3> GetWristTrajectory(void) const{
        return wrist_position_;
    }

private:
    std::vector<Vector3> elbow_position_;
    std::vector<Vector3> wrist_position_;
    const size_t number_of_vertices_;

};
}   // namespace collision
}   // namespace gtfo