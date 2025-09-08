#pragma once
#include "module.hpp"
#include "../math/pid.hpp"

namespace module {
    
struct flight_controller : module {

    struct rate_limit {double angular_velocity;};
    struct max_setpoint_deviation {double degrees; double roll_importance_multiplier;};
    struct artificial_stability {double angular_velocity_per_v_squared;};
    struct aoa_limit {double degrees_start, degrees_end;};

    struct roll_controller  {pid pid_; rate_limit rate_limit_;};
    struct pitch_controller {pid pid_; rate_limit rate_limit_; aoa_limit aoa_limit_; artificial_stability artificial_stability_;};
    struct yaw_controller   {pid pid_; rate_limit rate_limit_; aoa_limit aoa_limit_; artificial_stability artificial_stability_;};

    roll_controller  roll;
    pitch_controller pitch;
    yaw_controller   yaw;
    max_setpoint_deviation max_setpoint_deviation_;
    double artificial_stability_pitch, artificial_stability_yaw;
    Eigen::Quaterniond desired_rotation = Eigen::Quaterniond::Identity();
    flight_controller(
        roll_controller  roll_,
        pitch_controller pitch_,
        yaw_controller   yaw_,
        max_setpoint_deviation max_setpoint_deviation__,
        physics_object::object* parent);
    void update(physics_object::object* parent) override;
};

}