//----------------------------------------------------------------------------------------------------
// File: VirtualTunnelWrist.hpp
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
class VirtualTunnelWrist : public EntityPointTunnel<Scalar>{
public:

    using Vector3 = typename EntityPointTunnel<Scalar>::Vector3;
  
    VirtualTunnelWrist(const TunnelParameters &tunnel_parameters)  // the vertices are actually dots along the reference trajectory
    :   EntityPointTunnel<Scalar>(true),
        number_of_vertices_(tunnel_parameters.num_of_points)
    {
        // generate trajectory in joint space first
        Eigen::Matrix<double, 1, Eigen::Dynamic> pct = Eigen::VectorXd::LinSpaced(tunnel_parameters.num_of_points, 0.0, 1.0);
        Eigen::Matrix<double, 4, Eigen::Dynamic> pct_replicated = pct.replicate(4, 1);
        Eigen::Matrix<double, 4, Eigen::Dynamic> theta;
        Eigen::Matrix<double, 4, Eigen::Dynamic> start_matrix = tunnel_parameters.start_configuration.replicate(1, tunnel_parameters.num_of_points);
        Eigen::Matrix<double, 4, Eigen::Dynamic> end_matrix = tunnel_parameters.end_configuration.replicate(1, tunnel_parameters.num_of_points);

        Eigen::Matrix<double, 4, Eigen::Dynamic> pct3_replicated = pct_replicated.array() * pct_replicated.array() * pct_replicated.array(); // pct^3
        Eigen::Matrix<double, 4, Eigen::Dynamic> pct4_replicated = pct3_replicated.array() * pct_replicated.array(); // pct^4
        Eigen::Matrix<double, 4, Eigen::Dynamic> pct5_replicated = pct4_replicated.array() * pct_replicated.array();

        theta = start_matrix.array() + 
        (start_matrix.array() - end_matrix.array()) * 
        (15 * pct4_replicated.array() - 6 * pct5_replicated.array() - 10 * pct3_replicated.array());

        Eigen::RowVectorXd t1 = theta.row(0);
        Eigen::RowVectorXd t2 = theta.row(1);
        Eigen::RowVectorXd t3 = theta.row(2);
        Eigen::RowVectorXd t4 = theta.row(3);

        
        Eigen::RowVectorXd wrist_x = tunnel_parameters.lf * (t4.array().sin() * (t1.array().cos() * t3.array().sin() + t3.array().cos() * t1.array().sin() * t2.array().sin()) - t2.array().cos() * t4.array().cos() * t1.array().sin()) - tunnel_parameters.lu * t2.array().cos() * t1.array().sin();
        Eigen::RowVectorXd wrist_y = tunnel_parameters.lf * (t4.array().cos() * t2.array().sin() + t2.array().cos() * t3.array().cos() * t4.array().sin()) + tunnel_parameters.lu * t2.array().sin();
        Eigen::RowVectorXd wrist_z = -tunnel_parameters.lf * (t4.array().sin() * (t1.array().sin() * t3.array().sin() - t1.array().cos() * t3.array().cos() * t2.array().sin()) + t1.array().cos() * t2.array().cos() * t4.array().cos()) - tunnel_parameters.lu * t1.array().cos() * t2.array().cos();

        Eigen::Matrix<double, 3, Eigen::Dynamic> wrist(3, tunnel_parameters.num_of_points);
        wrist.row(0) = wrist_x;
        wrist.row(1) = wrist_y;
        wrist.row(2) = wrist_z;

        std::vector<Eigen::Vector3d> wrist_position;
    
        for (int i = 0; i < wrist.cols(); ++i) {
            wrist_position.push_back(wrist.col(i)); 
        }

        UpdateVertices(wrist_position);
    }

    void UpdateVirtualState() override {
    }

    void UpdateVertices(const std::vector<Vector3>& vertices) override{
        assert(vertices.size() == number_of_vertices_);
        assert(vertices.size() >= 1);
        EntityPointTunnel<Scalar>::vertices_ = vertices;
    }

private:

    const size_t number_of_vertices_;

};
}   // namespace collision
}   // namespace gtfo