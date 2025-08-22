// top of cpp marker
#include "collider.hpp"
#include "../renderer/mesh.hpp"


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
std::optional<vector::worldspace> collider::get_collision_position_model_to_model(collider& c, std::vector<vector::worldspace> intersection_points) {
    vector::worldspace line_segment_position_sum = vector::worldspace(0, 0, 0);
    for (vector::worldspace& point : intersection_points) {
        line_segment_position_sum += point;
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
    return line_segment_position_sum / intersection_points.size();
}
std::optional<std::vector<vector::worldspace>> collider::get_collision_normal_model_to_model(collider& c, std::vector<vector::worldspace> intersection_points) {
    
    /*
    Collisions are done by doing Principal Component Analysis on a vector of intersection points.

    The points are ideally distributed in a circle/oval shape. The direction that is effectively the "normal"
    of that vector of intersection points is the direction that of the 3rd (least) principal component.

    Eigenvalues represent how much of the variation in the vector of points is explained by each principal component.

    If the second eigenvalue is very small, that is a sign the data is almost linear and the third principal component
    could potentially be more parallel than normal with the ground.

    If the third eigenvalue is very large, approaching the second eigenvalue, then the data is too curved-in-3d
    to get a good third principal component out of, causing the same issue as above.
    */

    const double MINIMUM_RATIO_OF_EIGENVALUE_2_TO_1 = 0.001;
    const double MAXIMUM_RATIO_OF_EIGENVALUE_3_TO_2 = 0.1;

    //std::cout << "get_collision_normal_model_to_model: ";
    std::vector<double> input_points = {};
    std::vector<double> points_in_pca_space = {};
    if (intersection_points.size() < 3) {
        //std::cout << "intersection_points.size() = " << intersection_points.size() << " < 3, so a normal cannot be found\n";
        return std::nullopt;
    }
    for (vector::worldspace& point : intersection_points) {
        input_points.push_back(point.x());
        input_points.push_back(point.y());
        input_points.push_back(point.z());

        //std::cout << "ls.line_.direction.norm(): " << ls.line_.direction.norm() << "\n";
    }
    size_t dimensions = 3;
    size_t pca_output_count = 3; // max allowed for 3d
    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> principal_components_and_eigenvalues_optional = 
        math::pca(input_points, dimensions, points_in_pca_space, pca_output_count, math::PCA_ALG::COV, math::DATA_NORM::MEAN, false);
    
    if (!principal_components_and_eigenvalues_optional) return std::nullopt;

    Eigen::MatrixXd principal_components = principal_components_and_eigenvalues_optional.value().first;
    Eigen::VectorXd eigenvalues = principal_components_and_eigenvalues_optional.value().second;

    ///*
    for (int i = 0; i < points_in_pca_space.size(); i += 3) {
        vector::worldspace pca_component = {points_in_pca_space[i], points_in_pca_space[i+1], points_in_pca_space[i+2]};
        pca_component *= 1000;
        printf("points_in_pca_space[%i] x 1000: %s (magnitude %e)\n", i/3, pca_component.str().c_str(), pca_component.norm());
    }
    //*/

    std::cout << "principal_components:\n" << principal_components << "\ndone\n";
    
    std::cout << "eigenvalues:\n" << eigenvalues << "\ndone\n";

    vector::worldspace collision_plane_a = principal_components.cast<double>().col(0);
    vector::worldspace collision_plane_b = principal_components.cast<double>().col(1);
    vector::worldspace collision_normal  = principal_components.cast<double>().col(2);
    if (collision_normal.squaredNorm() == 0) {
        //std::cout << "collision_normal magnitude is 0, so a normal cannot be found\n";
        return std::nullopt;
    }

    double ratio_of_eigenvalue_2_to_1 = eigenvalues.row(1).value() / eigenvalues.row(0).value();
    if (ratio_of_eigenvalue_2_to_1 < MINIMUM_RATIO_OF_EIGENVALUE_2_TO_1) {
        std::cout << "ratio_of_eigenvalue_2_to_1 (" << ratio_of_eigenvalue_2_to_1 << ") < " <<
                     "MINIMUM_RATIO_OF_EIGENVALUE_2_TO_1 (" << MINIMUM_RATIO_OF_EIGENVALUE_2_TO_1 << ")\n";
        return std::nullopt;
    }

    double ratio_of_eigenvalue_3_to_2 = eigenvalues.row(2).value() / eigenvalues.row(1).value();
    if (ratio_of_eigenvalue_3_to_2 > MAXIMUM_RATIO_OF_EIGENVALUE_3_TO_2) {
        std::cout << "ratio_of_eigenvalue_3_to_2 (" << ratio_of_eigenvalue_3_to_2 << ") < " <<
                     "MAXIMUM_RATIO_OF_EIGENVALUE_3_TO_2 (" << MAXIMUM_RATIO_OF_EIGENVALUE_3_TO_2 << ")\n";
        //return std::nullopt;
    }

    ///*
    printf("pca_1: %s (magnitude %e)\n", collision_plane_a.str().c_str(), collision_plane_a.norm());
    printf("pca_2: %s (magnitude %e)\n", collision_plane_b.str().c_str(), collision_plane_b.norm());
    printf("pca_1 x pca_2 (collision_normal): %s (magnitude %e)\n", collision_normal.str().c_str(), collision_normal.norm());
    //*/
    //globals::timer_.record("collision data normal");
    return std::make_optional<std::vector<vector::worldspace>>({collision_plane_a, collision_plane_b, collision_normal});
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

namespace {

std::vector<vector::worldspace> to_roughly_equidistant_points(std::vector<line_segment> line_segments) {
    const int POINT_COUNT = 100;
    double length_sum = 0;
    for (line_segment& ls : line_segments) length_sum += ls.length;
    double points_per_meter = POINT_COUNT / length_sum;

    std::vector<vector::worldspace> output;

    for (line_segment& ls : line_segments) {
        double points_ = ls.length * points_per_meter;
        int points = round(points_);

        if (points == 0) continue;

        vector::worldspace start = ls.line_.origin;
        vector::worldspace end = ls.line_.point_along_line(ls.length);

        for (int i = 0; i < points; i++) {
            double fraction = i * 1.0 / points;
            vector::worldspace position = (1-fraction) * start + (fraction) * end;
            output.push_back(position);
        }
    }
    return output;
}

}

std::optional<collision_data> collider::get_collision_data(collider& c) {
    if (type == model_collider && c.type == model_collider) {
        auto line_segments = _get_line_segments_of_intersection_model_to_model(c);
        std::vector<vector::worldspace> intersection_points = to_roughly_equidistant_points(line_segments);
        return collision_data::optional(
            get_collision_position_model_to_model(c, intersection_points),
            get_collision_normal_model_to_model(c, intersection_points),
            intersection_points
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