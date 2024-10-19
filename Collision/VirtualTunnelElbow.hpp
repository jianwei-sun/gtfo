//----------------------------------------------------------------------------------------------------
// File: VirtualTunnelElbow.hpp
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

template<typename Scalar = double>
class VirtualTunnelElbow : public EntityPointTunnel<Scalar>{
public:

    using Vector3 = typename EntityPointTunnel<Scalar>::Vector3;
  
    VirtualTunnelElbow(const TunnelParameters &tunnel_parameters)  // the vertices are actually dots along the reference trajectory
        :   EntityPointTunnel<Scalar>(std::vector<Vector3>{Vector3::Zero()}, true),
        number_of_vertices_(tunnel_parameters.num_of_points)
    {
        // generate trajectory in joint space first
        Eigen::Matrix<double, Eigen::Dynamic, 1> pct = Eigen::VectorXd::LinSpaced(tunnel_parameters.num_of_points, 0.0, 1.0);
        Eigen::Matrix<double, 4, Eigen::Dynamic> theta;
        Eigen::Matrix<double, 4, Eigen::Dynamic> start_matrix = start_configuration_.replicate(1, tunnel_parameters.num_of_points);
        Eigen::Matrix<double, 4, Eigen::Dynamic> end_matrix = end_configuration_.replicate(1, tunnel_parameters.num_of_points);
        Eigen::ArrayXd pct_array = pct.array();
        Eigen::ArrayXd pct3 = pct_array * pct_array * pct_array; // pct^3
        Eigen::ArrayXd pct4 = pct3 * pct_array; // pct^4
        Eigen::ArrayXd pct5 = pct4 * pct_array;
        Eigen::ArrayXd pct4_replicated = pct4.replicate(4, 1);
        Eigen::ArrayXd pct5_replicated = pct5.replicate(4, 1);
        Eigen::ArrayXd pct3_replicated = pct3.replicate(4, 1);  
    
        theta = (start_matrix.array() + 
        (start_matrix.array() - end_matrix.array()) * 
        (15 * pct4_replicated - 6 * pct5_replicated - 10 * pct3_replicated)).matrix();

        Eigen::RowVectorXd t1 = theta.row(0);
        Eigen::RowVectorXd t2 = theta.row(1);
        Eigen::RowVectorXd t3 = theta.row(2);
        Eigen::RowVectorXd t4 = theta.row(3);

        Eigen::RowVectorXd elbow_x = -tunnel_parameters.lu * (t2.array().cos() * t1.array().sin());
        Eigen::RowVectorXd elbow_y = tunnel_parameters.lu * t2.array().sin();
        Eigen::RowVectorXd elbow_z = -tunnel_parameters.lu * (t1.array().cos() * t2.array().cos());
        
        Eigen::MatrixXd elbow;
        elbow.col(0) = elbow_x.transpose();
        elbow.col(1) = elbow_y.transpose();
        elbow.col(2) = elbow_z.transpose();

        std::vector<Eigen::Vector3d> elbow_position;
    
        for (int i = 0; i < elbow.rows(); ++i) {
            elbow_position.push_back(elbow.row(i)); 
        }

        UpdateVertices(elbow_position);
    }

    void UpdateVertices(const std::vector<Vector3>& vertices) override{
        assert(vertices.size() == number_of_vertices_);
        EntityPointTunnel<Scalar>::vertices_ = vertices;
    }

private:

    const size_t number_of_vertices_;

};
}   // namespace collision
}   // namespace gtfo