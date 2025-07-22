#pragma once
#include "shapes.cpp"
#include "collider_type.h"

namespace collision {

	struct collider {
		vector::worldspace position, velocity;
		Eigen::Quaterniond rotation;
		collider_type type;
		std::vector<triangle> model_data_unrotated;
		std::vector<triangle> model_data;
        vector::worldspace bounding_box_max;
        vector::worldspace bounding_box_min;
        double bounding_box_width_squared;
        private:
		bool check_collision_point_to_point(collider& c) {
			return false;
		}
		bool check_collision_point_to_model(collider& c) {
		  	vector::worldspace relative_velocity = c.velocity - velocity;
		  	if (relative_velocity.squaredNorm() < 1.0) return false;
		  	relative_velocity *= constants::DELTA_T;
		  	line_segment path_to_check(position, position+relative_velocity);
		  	for (triangle& t : c.model_data) {
				if (t.is_intersecting_line_segment(path_to_check)) return true;
		  	}
		  	return false;
		}
        bool check_collision_bounding_boxes(collider& c) {
            return (bounding_box_max.x() > c.bounding_box_min.x() &&
                    bounding_box_min.x() < c.bounding_box_max.x() &&
                    bounding_box_max.y() > c.bounding_box_min.y() &&
                    bounding_box_min.y() < c.bounding_box_max.y() &&
                    bounding_box_max.z() > c.bounding_box_min.z() &&
                    bounding_box_min.z() < c.bounding_box_max.z());
        }
        bool _check_collision_model_to_model_triangle(collider& c) {
            // the triangle approach is faster but cannot account for objects going through each other in under the duration of a tick
            if (!check_collision_bounding_boxes(c)) return false;

            for (triangle& tri1 : model_data) {
				for (triangle& tri2 : c.model_data) {
				  if (tri1.is_intersecting_triangle(tri2)) return true;
				}
		  	}
            return false;
        }
        bool _check_collision_model_to_model_prism(collider& c) {
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
		bool check_collision_model_to_model(collider& c) {
            return _check_collision_model_to_model_prism(c);
		}
        vector::worldspace get_point_of_collision_point_to_model(collider& c) {
            // just the raw position is good enough for things like missile and bullet hits  
            return position;
            //double min_distance = 0;
            //ray r = ray(position, velocity);
            //for (triangle& tri : c.model_data) {
            //    if (!tri.is_intersecting_ray(r)) continue;
            //    double distance = tri()
            //}
        }
        std::optional<vector::worldspace> get_point_of_collision_model_to_model(collider& c) {
            vector::worldspace line_segment_position_sum = vector::worldspace(0, 0, 0);
            double line_segment_length_sum = 0;
            for (triangle& tri1 : model_data) {
                for (triangle& tri2 : c.model_data) {
                    if (!tri1.is_intersecting_triangle(tri2)) continue;
                    std::optional<line_segment> ls_optional = tri1.intersection(tri2);
                    if (!ls_optional) continue;
                    line_segment& ls = ls_optional.value();
                    line_segment_position_sum += ls.line_.origin * ls.length;
                    line_segment_position_sum += ls.line_.point_along_line(ls.length) * ls.length;
                    line_segment_length_sum += ls.length;
                }
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
            return line_segment_position_sum / (2 * line_segment_length_sum);
        }
		void rotate_model_data(vector::worldspace position_, Eigen::Quaterniond rotation_) {
		  	model_data.clear();
		  	for (int i = 0; i < model_data_unrotated.size(); i++) {
			model_data.push_back(triangle(
				static_cast<vector::localspace>(model_data_unrotated[i].points[0]).to_worldspace_positional(rotation_, position_),
				static_cast<vector::localspace>(model_data_unrotated[i].points[1]).to_worldspace_positional(rotation_, position_),
				static_cast<vector::localspace>(model_data_unrotated[i].points[2]).to_worldspace_positional(rotation_, position_)
			));
		  	}
		}
        void set_bounding_box() {
            double max = std::numeric_limits<double>::max();
            double min = std::numeric_limits<double>::min();
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

            bounding_box_width_squared = (bounding_box_max - bounding_box_min).squaredNorm();
        }
        public:
		collider() {
		  type = point_collider;
		}
		collider(std::vector<triangle> model_data_unrotated_) {
		  type = model_collider;
		  model_data_unrotated = model_data_unrotated_;
		}
		void update(vector::worldspace position_, vector::worldspace velocity_, Eigen::Quaterniond rotation_) {
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
			  	break;
		  	}
		}
		bool check_collision(collider& c) {
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
        std::optional<vector::worldspace> get_point_of_collision(collider& c) {
		  	if (type == model_collider && c.type == model_collider) {
				return get_point_of_collision_model_to_model(c);
		  	}
		  	if (type == point_collider && c.type == model_collider) {
				return get_point_of_collision_point_to_model(c);
		  	}
		  	if (type == model_collider && c.type == point_collider) {
				return c.get_point_of_collision_point_to_model(*this);
		  	}
		  	std::cout << "check_collision called on two point colliders\n";
			std::abort();
		}
	};
}