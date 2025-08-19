#pragma once
#include "shapes.cpp"
#include "collider_type.h"
#include "collision_data.cpp"
#include "../eigen_pca/eigen-pca.hpp"
#include "../renderer/mesh.h"


namespace collision {

    bool collider::check_collision_point_to_point(collider& c) {
        return false;
    }

    bool collider::check_collision_point_to_model(collider& c) {
        vector::worldspace relative_velocity = c.velocity - velocity;
        if (relative_velocity.squaredNorm() < 1.0) return false;
        relative_velocity *= constants::DELTA_T;
        line_segment path_to_check(position, position+relative_velocity);
        for (triangle& t : c.model_data) {
            if (t.is_intersecting_line_segment(path_to_check)) return true;
        }
        return false;
    }
    bool collider::check_collision_bounding_boxes(collider& c) {
        return (bounding_box_max.x() > c.bounding_box_min.x() &&
                bounding_box_min.x() < c.bounding_box_max.x() &&
                bounding_box_max.y() > c.bounding_box_min.y() &&
                bounding_box_min.y() < c.bounding_box_max.y() &&
                bounding_box_max.z() > c.bounding_box_min.z() &&
                bounding_box_min.z() < c.bounding_box_max.z());
    }
    bool collider::_check_collision_model_to_model_triangle(collider& c) {
        // the triangle approach is faster but cannot account for objects going through each other in under the duration of a tick
        if (!check_collision_bounding_boxes(c)) return false;

        for (triangle& tri1 : model_data) {
            for (triangle& tri2 : c.model_data) {
                //std::cout << "tri1 centroid: " << tri1.centroid.str() << "\n";
                //std::cout << "tri2 centroid: " << tri2.centroid.str() << "\n";
                //std::cout << "tri1 radius: " << tri1.radius << "\n";
                //std::cout << "tri2 radius: " << tri2.radius << "\n";
                //std::cout << "dist^2: " << (tri1.centroid - tri2.centroid).squaredNorm() << "\n";
                //std::cout << "radius^2: " << (tri1.radius + tri2.radius) * (tri1.radius + tri2.radius) << "\n";
                if ((tri1.centroid - tri2.centroid).squaredNorm() > (tri1.radius + tri2.radius) * (tri1.radius + tri2.radius)) continue;
                if (!tri1.is_intersecting_triangle_bounding_box(tri2)) continue;
                if (tri1.is_intersecting_triangle(tri2)) return true;
            }
        }
        return false;
    }
    bool collider::_check_collision_model_to_model_prism(collider& c) {
        // the prism approach is slower so raw tri-to-tri can be done instead for performance
        vector::worldspace relative_velocity = c.velocity - velocity;
        bool relative_velocity_is_zero = relative_velocity.squaredNorm() == 0;
        relative_velocity *= constants::DELTA_T;

        // a simple bounding box intersection check doesn't account for fly-through collisions, so do this instead
        double loose_max_distance_squared = bounding_box_width_squared + c.bounding_box_width_squared + relative_velocity.squaredNorm();
        double distance_squared = (bounding_box_min - c.bounding_box_min).squaredNorm();
        if (distance_squared > loose_max_distance_squared) return false;

        for (triangle& t : model_data) {
            triangular_prism prism = triangular_prism(t, relative_velocity);
            for (triangle& tri1 : c.model_data) {
                for (triangle& tri2 : prism.faces) {
                    if (tri1.is_intersecting_triangle(tri2)) return true;
                }
                // check for the triangle being entirely within the prism
                if (!relative_velocity_is_zero && prism.surrounds_point(tri1.points[0])) return true;
            }
        }
        return false;
    }
    bool collider::check_collision_model_to_model(collider& c) {
        return _check_collision_model_to_model_triangle(c);
    }
    vector::worldspace collider::get_collision_position_point_to_model(collider& c) {
        // just the raw position is good enough for things like missile and bullet hits, rather than factoring in the sub-tick movement
        return position;
        //double min_distance = 0;
        //ray r = ray(position, velocity);
        //for (triangle& tri : c.model_data) {
        //    if (!tri.is_intersecting_ray(r)) continue;
        //    double distance = tri()
        //}
    }
    std::optional<vector::worldspace> collider::get_collision_normal_point_to_model(collider& c) {
        ray r = static_cast<ray>(line(position, velocity));
        std::optional<triangle*> minimum_time_triangle = std::nullopt;
        double minimum_time = std::numeric_limits<double>::max();
        for (triangle& t : c.model_data) {
            double time = t.is_intersecting_ray(r);
            if (time = -1.0) continue;
            if (time < minimum_time) {
                minimum_time_triangle = &t;
                minimum_time = time;
            }
        }
        if (!minimum_time_triangle) return vector::worldspace(0, 0, 1);//std::nullopt;
        return minimum_time_triangle.value()->get_normal();
    }
    std::vector<line_segment> collider::_get_line_segments_of_intersection_model_to_model(collider& c) {
        //std::cout << "_get_line_segments_of_intersection_model_to_model:\n";
        std::vector<line_segment> output;
        //std::cout << "model_data length: " << model_data.size() << "\n";
        //std::cout << "c.model_data length: " << c.model_data.size() << "\n";
        for (triangle& tri1 : model_data) {
            for (triangle& tri2 : c.model_data) {
                //std::cout << "tri1: [" << tri1.points[0].str() << ", " << tri1.points[1].str() << ", " << tri1.points[2].str() << "]\n";
                //std::cout << "tri2: [" << tri2.points[0].str() << ", " << tri2.points[1].str() << ", " << tri2.points[2].str() << "]\n";
                if ((tri1.centroid - tri2.centroid).squaredNorm() > (tri1.radius + tri2.radius) * (tri1.radius + tri2.radius)) continue;
                if (!tri1.is_intersecting_triangle_bounding_box(tri2)) continue;
                if (!tri1.is_intersecting_triangle(tri2)) continue;
                
                //std::cout << "tri1 and tri2 are intersecting\n";
                std::optional<line_segment> ls_optional = tri1.intersection(tri2);
                if (!ls_optional) {
                    //std::cout << "no line_segment found\n";
                    continue;
                }
                //std::cout << "line_segment found\n";
                //std::cout << "pushing back line_segment\n";
                output.push_back(ls_optional.value());
            }
        }
        //std::cout << "_get_line_segments_of_intersection_model_to_model done\n";
        //globals::timer_.record("collision data line segments");
        return output;
    }
    std::optional<vector::worldspace> collider::get_collision_position_model_to_model(collider& c, std::vector<line_segment> intersections) {
        vector::worldspace line_segment_position_sum = vector::worldspace(0, 0, 0);
        double line_segment_length_sum = 0;
        for (line_segment& ls : intersections) {
            line_segment_position_sum += ls.line_.origin * ls.length;
            line_segment_position_sum += ls.line_.point_along_line(ls.length) * ls.length;
            line_segment_length_sum += ls.length;
        }
        if (line_segment_length_sum == 0) {
            //std::cout << "get_point_of_collision_model_to_model(collider& c): line_segment_length_sum == 0\n";
            return std::nullopt;
        }
        if (
            std::isnan(line_segment_position_sum.x()) ||
            std::isnan(line_segment_position_sum.y()) ||
            std::isnan(line_segment_position_sum.z())
        ) {
            //std::cout << "get_point_of_collision_model_to_model(collider& c): nan in line_segment_position_sum\n";
            return std::nullopt;
        }
        //globals::timer_.record("collision data position");
        return line_segment_position_sum / (2 * line_segment_length_sum);
    }
    std::optional<vector::worldspace> collider::get_collision_normal_model_to_model(collider& c, std::vector<line_segment> intersections) {
        //std::cout << "get_collision_normal_model_to_model: ";
        std::vector<float> input_points;
        std::vector<float> pca_output;
        if (intersections.size() < 2) {
            //std::cout << "intersections.size() = " << intersections.size() << " < 2, so a normal cannot be found\n";
            return std::nullopt;
        }
        for (line_segment& ls : intersections) {
            input_points.push_back(ls.line_.origin.x());
            input_points.push_back(ls.line_.origin.y());
            input_points.push_back(ls.line_.origin.z());

            input_points.push_back(ls.line_.origin.x() + ls.line_.point_along_line(ls.length).x());
            input_points.push_back(ls.line_.origin.y() + ls.line_.point_along_line(ls.length).y());
            input_points.push_back(ls.line_.origin.z() + ls.line_.point_along_line(ls.length).z());

            //std::cout << "ls.line_.direction.norm(): " << ls.line_.direction.norm() << "\n";
        }
        size_t dimensions = 3;
        size_t pca_output_count = 3; // max allowed for 3d
        math::pca(input_points, dimensions, pca_output, pca_output_count);
        
        vector::worldspace collision_plane_a = vector::worldspace( // the 1st vector of pca_output
            pca_output[0],
            pca_output[1],
            pca_output[2]
        );
        vector::worldspace collision_plane_b = vector::worldspace( // the 2nd vector of pca_output
            pca_output[3],
            pca_output[4],
            pca_output[5]
        );
        vector::worldspace collision_normal = collision_plane_a.cross(collision_plane_b);
        if (collision_normal.squaredNorm() == 0) {
            //std::cout << "collision_normal magnitude is 0, so a normal cannot be found\n";
            return std::nullopt;
        }
        collision_normal.normalize();
        //std::cout << "collision_normal: " << collision_normal.str() << "\n";
        //globals::timer_.record("collision data normal");
        return collision_normal;
    }
    void collider::rotate_model_data(vector::worldspace position_, Eigen::Quaterniond rotation_) {
        model_data.clear();
        for (int i = 0; i < model_data_unrotated.size(); i++) {
        model_data.push_back(triangle(
            static_cast<vector::localspace>(model_data_unrotated[i].points[0]).to_worldspace_positional(rotation_, position_),
            static_cast<vector::localspace>(model_data_unrotated[i].points[1]).to_worldspace_positional(rotation_, position_),
            static_cast<vector::localspace>(model_data_unrotated[i].points[2]).to_worldspace_positional(rotation_, position_)
        ));
        }
    }
    void collider::set_bounding_box() {
        double max = std::numeric_limits<double>::max();
        double min = std::numeric_limits<double>::lowest();
        bounding_box_min = vector::worldspace(max, max, max);
        bounding_box_max = vector::worldspace(min, min, min);
        
        for (triangle& t : model_data) {
            for(vector::worldspace& v : t.points) {
                bounding_box_min.x() = fmin(bounding_box_min.x(), v.x());
                bounding_box_min.y() = fmin(bounding_box_min.y(), v.y());
                bounding_box_min.z() = fmin(bounding_box_min.z(), v.z());
                
                bounding_box_max.x() = fmax(bounding_box_max.x(), v.x());
                bounding_box_max.y() = fmax(bounding_box_max.y(), v.y());
                bounding_box_max.z() = fmax(bounding_box_max.z(), v.z());
            }
        }

        //std::cout << "bounding_box_min: " << bounding_box_min.str() << "\n";
        //std::cout << "bounding_box_max: " << bounding_box_max.str() << "\n";

        bounding_box_width_squared = (bounding_box_max - bounding_box_min).squaredNorm();
    }
    collider::collider() {
        type = point_collider;
    }
    collider::collider(std::vector<triangle> model_data_unrotated_) {
        type = model_collider;
        model_data_unrotated = model_data_unrotated_;
    }
    collider::collider(mesh m) {
        type = model_collider;
        std::vector<triangle> model_data_unrotated_;
        std::vector<vector::worldspace> points;
        for (int index : m.indices) {
            points.push_back(vector::worldspace(
                m.vertices[index].x,
                m.vertices[index].y,
                m.vertices[index].z
            ));
        }
        for (int i = 0; i < points.size(); i += 3) {
            model_data_unrotated_.push_back(triangle(
                points[i],
                points[i+1],
                points[i+2]
            ));
        }
        model_data_unrotated = model_data_unrotated_;
    }
    void collider::update(vector::worldspace position_, vector::worldspace velocity_, Eigen::Quaterniond rotation_) {
        switch(type) {
        case point_collider:
            position = position_;
            velocity = velocity_;
            break;
        case model_collider:
            position = position_;
            velocity = velocity_;
            rotation = rotation_;
            rotate_model_data(position_, rotation_);
            set_bounding_box();
            for (triangle& t : model_data) t.update();
            break;
        }
    }
    bool collider::check_collision(collider& c) {
        if (type == point_collider && c.type == point_collider) {
            return false;
        }
        if (type == model_collider && c.type == model_collider) {
            return check_collision_model_to_model(c);
        }
        if (type == point_collider && c.type == model_collider) {
            return check_collision_point_to_model(c);
        }
        if (type == model_collider && c.type == point_collider) {
            return c.check_collision_point_to_model(*this);
        }
        return false;
    }
    std::optional<collision_data> collider::get_collision_data(collider& c) {
        if (type == model_collider && c.type == model_collider) {
            auto line_segments = _get_line_segments_of_intersection_model_to_model(c);
            return collision_data::optional(
                get_collision_position_model_to_model(c, line_segments),
                get_collision_normal_model_to_model(c, line_segments),
                line_segments
            );
        }
        if (type == point_collider && c.type == model_collider) {
            return collision_data::optional(
                get_collision_position_point_to_model(c),
                get_collision_normal_point_to_model(c)
            );
        }
        if (type == model_collider && c.type == point_collider) {
            return collision_data::optional(
                c.get_collision_position_point_to_model(*this),
                c.get_collision_normal_point_to_model(*this)
            );
        }
        //std::cout << "check_collision called on two point colliders\n";
        std::abort();
    }
}