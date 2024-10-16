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

template<typename Scalar = double>
class VirtualTunnel : public EntityPointTunnel<Scalar>{
public:

    using Vector3 = typename EntityPointTunnel<Scalar>::Vector3;
  
    VirtualTunnel(const std::vector<Vector3>& vertices)  // the vertices are actually dots along the reference trajectory
        :   EntityPointTunnel<Scalar>(vertices, true)

    {}

    
private:


};

}   // namespace collision
}   // namespace gtfo