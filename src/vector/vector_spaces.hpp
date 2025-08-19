#pragma once
#include "../constants/constants.cpp"


namespace vector {

struct worldspace : public Eigen::Vector3d {
    using Eigen::Vector3d::Vector3d;
    localspace to_localspace(Eigen::Quaterniond q);
    localspace to_localspace_positional(Eigen::Quaterniond q, worldspace position);
    std::string str();
    worldspace add_angular_velocity(worldspace relative_position, worldspace angular_velocity);
};

struct localspace : public Eigen::Vector3d {
    using Eigen::Vector3d::Vector3d;
    worldspace to_worldspace(Eigen::Quaterniond q);
    worldspace to_worldspace_positional(Eigen::Quaterniond q, worldspace position);
    void clamp();
    void clamp(double num);
    std::string str();
};

struct scopespace {

    private:

    localspace v;

    public:

    double& distance();
    const double& distance() const;

    double& scope_x();
    const double& scope_x() const;

    double& scope_y();
    const double& scope_y() const;

    scopespace& operator+=(const scopespace& s);
    scopespace& operator/=(double num);
    scopespace();
    std::string str();
};

localspace worldspace::to_localspace(Eigen::Quaterniond q);
localspace worldspace::to_localspace_positional(Eigen::Quaterniond q, worldspace position);
worldspace localspace::to_worldspace(Eigen::Quaterniond q);
worldspace localspace::to_worldspace_positional(Eigen::Quaterniond q, worldspace position);

}