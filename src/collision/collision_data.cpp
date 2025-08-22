// top of cpp marker
#include "collision_data.hpp"

namespace collision {

collision_data::collision_data(vector::worldspace position_, vector::worldspace normal_) {
    position = position_;
    normal = normal_;
}

std::optional<collision_data> collision_data::optional(std::optional<vector::worldspace> position_, std::optional<vector::worldspace> normal_) {
    if (!position_) return std::nullopt;
    if (!normal_) return std::nullopt;
    return collision_data(position_.value(), normal_.value());
}

collision_data::collision_data(vector::worldspace position_, std::vector<vector::worldspace> pca_components, std::vector<vector::worldspace> intersection_points_) {
    position = position_;
    pca_primary = pca_components[0];
    pca_secondary = pca_components[1];
    normal = pca_components[2];
    intersection_points = intersection_points_;
}

std::optional<collision_data> collision_data::optional(std::optional<vector::worldspace> position_, std::optional<std::vector<vector::worldspace>> pca_components, std::vector<vector::worldspace> intersection_points_) {
    if (!position_) return std::nullopt;
    if (!pca_components) return std::nullopt;
    return collision_data(position_.value(), pca_components.value(), intersection_points_);
}

}