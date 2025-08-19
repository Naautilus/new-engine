#pragma once

#include "../vector/vector_spaces.hpp"
#include "shapes.hpp"

namespace collision {
    
struct collision_data {
    vector::worldspace position;
    vector::worldspace normal;
    std::vector<line_segment> debug_line_segments;
    collision_data(vector::worldspace position_, vector::worldspace normal_);
    static std::optional<collision_data> optional(std::optional<vector::worldspace> position_, std::optional<vector::worldspace> normal_);
    collision_data(vector::worldspace position_, vector::worldspace normal_, std::vector<line_segment> debug_line_segments_);
    static std::optional<collision_data> optional(std::optional<vector::worldspace> position_, std::optional<vector::worldspace> normal_, std::vector<line_segment> debug_line_segments_);
};

}