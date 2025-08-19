// top of cpp marker
#include "physics_state.hpp"

vector::worldspace physics_state::velocity_at_point(vector::worldspace position_absolute) {
    return velocity.add_angular_velocity(position_absolute - position, angular_velocity);
}