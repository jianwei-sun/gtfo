//----------------------------------------------------------------------------------------------------
// File: SoftBoundPhysics.hpp
// Desc: Physics logic for interacting with soft bounds
//----------------------------------------------------------------------------------------------------
#pragma once

// Project-specific
#include "BoundExpression.hpp"

namespace gtfo
{

    // Returns the resotring force in N-dimensional space to push the model back into bounds
    static VectorN RestoringForce(const BoundExpression<Dimensions, Scalar> &bound, const VectorN &point)
    {
        // TODO: impliment logic for unidimensonal damping and spring
    }

}