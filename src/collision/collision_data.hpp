#pragma once

#include "../vector/vector_spaces.hpp"
#include "shapes.hpp"

namespace collision {
    
struct collision_data {
    vector::worldspace position;
    vector::worldspace normal; // pca_tertiary
    std::optional<vector::worldspace> pca_primary;
    std::optional<vector::worldspace> pca_secondary;
    std::optional<std::vector<vector::worldspace>> intersection_points;

    collision_data(vector::worldspace position_, vector::worldspace normal_);
    static std::optional<collision_data> optional(std::optional<vector::worldspace> position_, std::optional<vector::worldspace> normal_);

    collision_data(vector::worldspace position_, std::vector<vector::worldspace> pca_components, std::vector<vector::worldspace> intersection_points_);
    static std::optional<collision_data> optional(std::optional<vector::worldspace> position_, std::optional<std::vector<vector::worldspace>> pca_components, std::vector<vector::worldspace> intersection_points_);
};

}