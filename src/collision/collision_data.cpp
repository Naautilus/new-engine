#pragma once

#include "../vector/vector_spaces.cpp"

namespace collision {
    struct collision_data {
        vector::worldspace position;
        vector::worldspace normal;
        std::vector<line_segment> debug_line_segments;
        collision_data(vector::worldspace position_, vector::worldspace normal_) {
            position = position_;
            normal = normal_;
        }
        static std::optional<collision_data> optional(std::optional<vector::worldspace> position_, std::optional<vector::worldspace> normal_) {
            if (!position_) return std::nullopt;
            if (!normal_) return std::nullopt;
            return collision_data(position_.value(), normal_.value());
        }

        collision_data(vector::worldspace position_, vector::worldspace normal_, std::vector<line_segment> debug_line_segments_) {
            position = position_;
            normal = normal_;
            debug_line_segments = debug_line_segments_;
        }
        static std::optional<collision_data> optional(std::optional<vector::worldspace> position_, std::optional<vector::worldspace> normal_, std::vector<line_segment> debug_line_segments_) {
            if (!position_) return std::nullopt;
            if (!normal_) return std::nullopt;
            return collision_data(position_.value(), normal_.value(), debug_line_segments_);
        }
    };
}