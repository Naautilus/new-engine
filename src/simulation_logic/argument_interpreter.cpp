#include "argument_interpreter.hpp"

void interpret_arguments(std::vector<std::string> args) {
    if (std::find(args.begin(), args.end(), "-missile-cam") != args.end()) globals::MISSILE_CAMERA = true;
    if (std::find(args.begin(), args.end(), "-show-collision-debugging") != args.end()) globals::SHOW_COLLISION_DEBUGGING = true;
    if (std::find(args.begin(), args.end(), "-pause-on-collision") != args.end()) globals::PAUSE_ON_COLLISION = true;
    if (std::find(args.begin(), args.end(), "-show-missile-acceleration") != args.end()) globals::SHOW_MISSILE_ACCELERATION = true;
    if (std::find(args.begin(), args.end(), "-verbose-log") != args.end()) globals::VERBOSE_LOG = true;
    if (std::find(args.begin(), args.end(), "-show-fps") != args.end()) globals::SHOW_FPS = true;
    if (std::find(args.begin(), args.end(), "-fire-bullets") != args.end()) globals::BULLETS_INSTEAD_OF_MISSILES= true;
    if (std::find(args.begin(), args.end(), "-no-flight-controller") != args.end()) globals::NO_FLIGHT_CONTROLLER = true;
}