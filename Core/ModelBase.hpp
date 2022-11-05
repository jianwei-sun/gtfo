//----------------------------------------------------------------------------------------------------
// File: ModelBase.hpp
// Desc: base class for dynamics models
//----------------------------------------------------------------------------------------------------
#pragma once

// Standard libraries includes
#include <cmath>
#include <type_traits>

// Third-party dependencies
#include <Eigen/Dense>

namespace gtfo{

template<typename Scalar = double>
struct ParametersBase{
    Scalar dt;

    ParametersBase(const Scalar& dt) : dt(dt){
        assert(dt > 0.0);
    }
};

template<unsigned int Dimensions, typename Parameters, typename Scalar = double>
class ModelBase{
public:
    static_assert(Dimensions > 0, "Dimensions must be at least 1.");
    static_assert(std::is_floating_point_v<Scalar>, "Template argument Scalar must be a floating-point type.");

    using VectorN = Eigen::Matrix<Scalar, Dimensions, 1>;
    using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
    using Matrix2 = Eigen::Matrix<Scalar, 2, 2>;
    using PossibleScalar = std::conditional_t<Dimensions == 1, Scalar, VectorN>;

    ModelBase()
        :   A_continuous_(Matrix2::Zero()),
            B_continuous_(Vector2::Zero()),
            A_discrete_(Matrix2::Zero()),
            B_discrete_(Vector2::Zero()),
            position_(VectorN::Zero()),
            velocity_(VectorN::Zero()),
            acceleration_(VectorN::Zero()){}

    virtual void SetParameters(const Parameters& parameters) = 0;

    // Propagate the dynamics forward by one time-step
    virtual void Step(const PossibleScalar& input){
        const Eigen::Matrix<Scalar, 2, Dimensions> state = (Eigen::Matrix<Scalar, 2, Dimensions>() << position_.transpose(), velocity_.transpose()).finished();
        const Eigen::Matrix<Scalar, 2, Dimensions> new_state = A_discrete_ * state + (input * B_discrete_.transpose()).transpose();
        position_ = new_state.row(0);
        velocity_ = new_state.row(1);
        acceleration_ = A_continuous_.row(1) * state + (input * B_continuous_.row(1).transpose()).transpose();
    }

    [[nodiscard]] inline const PossibleScalar& GetPosition() const {
        if constexpr(Dimensions == 1)
            return position_.value();
        else
            return position_; 
    }

    [[nodiscard]] inline const PossibleScalar& GetVelocity() const {
        if constexpr(Dimensions == 1)
            return velocity_.value();
        else
            return velocity_;
    }

    [[nodiscard]] inline const PossibleScalar& GetAcceleration() const {
        if constexpr(Dimensions == 1)
            return acceleration_.value();
        else
            return acceleration_;
    }

protected:
    Matrix2 A_continuous_;
    Vector2 B_continuous_;

    Matrix2 A_discrete_;
    Vector2 B_discrete_;

    VectorN position_;
    VectorN velocity_;
    VectorN acceleration_;
};

}   // namespace gtfo
