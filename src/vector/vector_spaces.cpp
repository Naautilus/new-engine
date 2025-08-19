// top of cpp marker
#include "vector_spaces.hpp"

namespace vector {

std::string worldspace::str() {
    std::string output = "{";
    output += std::format(constants::FORMAT_STRING_POSITION, x()) + ", ";
    output += std::format(constants::FORMAT_STRING_POSITION, y()) + ", ";
    output += std::format(constants::FORMAT_STRING_POSITION, z()) + "}";
    return output;
}
worldspace worldspace::add_angular_velocity(worldspace relative_position, worldspace angular_velocity) {
    return *this - angular_velocity.cross(relative_position);
}

void localspace::clamp() {
    this->x() = std::clamp(this->x(), -1.0, 1.0);
    this->y() = std::clamp(this->y(), -1.0, 1.0);
    this->z() = std::clamp(this->z(), -1.0, 1.0);
}
void localspace::clamp(double num) {
    this->x() = std::clamp(this->x(), -num, num);
    this->y() = std::clamp(this->y(), -num, num);
    this->z() = std::clamp(this->z(), -num, num);
}
std::string localspace::str() {
    std::string output = "{";
    output += std::format(constants::FORMAT_STRING_POSITION, x()) + ", ";
    output += std::format(constants::FORMAT_STRING_POSITION, y()) + ", ";
    output += std::format(constants::FORMAT_STRING_POSITION, z()) + "}";
    return output;
}

double& scopespace::distance() { return v[0]; }
const double& scopespace::distance() const { return v[0]; }

double& scopespace::scope_x() { return v[1]; }
const double& scopespace::scope_x() const { return v[1]; }

double& scopespace::scope_y() { return v[2]; }
const double& scopespace::scope_y() const { return v[2]; }

scopespace& scopespace::operator+=(const scopespace& s) {
    v += s.v;
    return *this;
}
scopespace& scopespace::operator/=(double num) {
    v /= num;
    return *this;
}

scopespace::scopespace() {
    v = localspace(0, 0, 0);
}

std::string scopespace::str() {
    std::string output = "{";
    output += std::format(constants::FORMAT_STRING_POSITION, distance()) + ", (";
    output += std::format(constants::FORMAT_STRING_UNIT, scope_x()) + ", ";
    output += std::format(constants::FORMAT_STRING_UNIT, scope_y()) + ")}";
    return output;
}

localspace worldspace::to_localspace(Eigen::Quaterniond q) {
    return q * (*this);
}

localspace worldspace::to_localspace_positional(Eigen::Quaterniond q, worldspace position) {
    return q * ((*this) - position);
}

worldspace localspace::to_worldspace(Eigen::Quaterniond q) {
    return q.inverse() * (*this);
}

worldspace localspace::to_worldspace_positional(Eigen::Quaterniond q, worldspace position) {
    return (q.inverse() * (*this)) + position;
}

}