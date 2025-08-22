// top of cpp marker
#include "constants.hpp"

namespace constants {

const double DELTA_T = 0.005;
const double LOG_INTERVAL = 0.005;
const double TIME_LIMIT = -1;
const double DAMAGE_MULTIPLIER = 2e-4;
const double UNCONTROLLABLE_HEALTH_FRACTION = 0.8;
const double SAFE_COLLISION_SPEED = 5;
const double CYLINDER_VERTICES = 8;
const double QUADRATIC_PLANET_CURVATURE_COEFFICIENT = 7.85e-8;// * 1e3; // the large multiplier is to make it extremely severe for testing purposes
const double STANDARD_GRAVITY = 9.81;

}