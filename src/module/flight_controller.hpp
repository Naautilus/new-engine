#pragma once
#include "module.hpp"
#include "../math/pid.hpp"

namespace module {
    
struct flight_controller : module {
    pid pid_roll, pid_pitch, pid_yaw;
    double max_roll_rate, max_pitch_rate, max_yaw_rate;
    Eigen::Quaterniond desired_rotation = Eigen::Quaterniond::Identity();
    flight_controller(
        pid pid_roll_, double max_roll_rate_, 
        pid pid_pitch_, double max_pitch_rate_, 
        pid pid_yaw_, double max_yaw_rate_,
        physics_object::object* parent);
    void update(physics_object::object* parent) override;
};

}