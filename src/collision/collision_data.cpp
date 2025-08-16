#pragma once

#include "../vector/vector_spaces.cpp"

namespace collision {
    struct collision_data {
        vector::worldspace position;
        vector::worldspace normal;
        collision_data(vector::worldspace position_, vector::worldspace normal_) {
            position = position_;
            normal = normal_;
        }
        static std::optional<collision_data> optional(std::optional<vector::worldspace> position_, std::optional<vector::worldspace> normal_) {
            if (!position_) {
                std::cout << "collision_data.cpp: no collision position\n";
            }
            if (!normal_) {
                std::cout << "collision_data.cpp: no collision normal\n";
            }
            if (!position_) {
                return std::nullopt;
            }
            if (!normal_) {
                return std::nullopt;
            }
            return collision_data(position_.value(), normal_.value());
        }
    };
}