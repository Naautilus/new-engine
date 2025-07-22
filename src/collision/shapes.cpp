#pragma once
#include "../vector/vector_spaces.cpp"
#include "../globals/globals.cpp"

namespace collision {

	struct line {
		vector::worldspace origin = vector::worldspace(0, 0, 0);
		vector::worldspace direction = vector::worldspace(1, 0, 0);
		line() {}
		line(vector::worldspace origin_, vector::worldspace direction_) {
		  	origin = origin_;
		  	direction = direction_;
		}
        double distance_along_line(vector::worldspace input_point) {
		  	input_point -= origin; // offset the input point to be relative to the triangle
		  	return input_point.dot(direction);
		}
        double distance_to_intersection(line& l) {
            // Assumes that the lines are intersecting
            double t = (l.origin - origin).cross(l.direction).norm() / direction.cross(l.direction).norm();
            return t;
        }
        vector::worldspace point_along_line(double t) {
            return origin + t * direction;
        }
	};

    struct ray : line {};

    struct plane {
        vector::worldspace origin = vector::worldspace(0, 0, 0);
		vector::worldspace direction = vector::worldspace(1, 0, 0);
        plane() {}
		plane(vector::worldspace origin_, vector::worldspace direction_) {
		  	origin = origin_;
		  	direction = direction_;
		}
        double distance_along_normal(vector::worldspace input_point) {
		  	input_point -= origin; // offset the input point to be relative to the triangle
		  	return input_point.dot(direction);
		}
        std::optional<line> line_of_intersection(plane& p) {
            // Assumes that the planes are not parallel

            if (fabs(fabs(direction.dot(p.direction)) - 1) < std::numeric_limits<double>::epsilon()) {
                std::cout << "line_of_intersection(plane& p): parallel planes\n";
                return std::nullopt;
            }
            line l;
            l.direction = direction.cross(p.direction);

            /*
            A linear solve will be done.
            the result of the solve is a vector X that satisfies all three conditions:

            dot(plane 1's normal, X) = dot(plane 1's normal, plane 1's origin).
            dot(plane 2's normal, X) = dot(plane 2's normal, plane 2's origin).
            dot(line's normal, X) = dot(line's normal, {0, 0, 0}).

            Each of these 3 conditions affirm that X is perpendicular to each plane/line's normal
            while being offset from {0,0,0} the same amount as that plane's origin,
            meaning it is on the plane and perpendicular to it.

            (The line is treated as a plane with a normal of its direction and an origin of {0, 0, 0}.)
            */

            Eigen::Matrix3d coefficients_matrix;
            coefficients_matrix.row(0) = direction;
            coefficients_matrix.row(1) = p.direction;
            coefficients_matrix.row(2) = l.direction;
            //std::cout << "direction: " << direction.transpose() << "\n";
            //std::cout << "p.direction: " << p.direction.transpose() << "\n";
            //std::cout << "l.direction: " << l.direction.transpose() << "\n";

            //for (int i = 0; i < 3; i++) {
            //    std::cout << "matrix[" << i << "]: " << coefficients_matrix.row(i).transpose() << "\n";
            //}
            
            Eigen::Vector3d constraint_vector;
            constraint_vector[0] = direction.dot(origin);
            constraint_vector[1] = p.direction.dot(p.origin);
            constraint_vector[2] = 0;
            //std::cout << "direction.dot(origin): " << direction.dot(origin) << "\n";
            //std::cout << "p.direction.dot(p.origin): " << p.direction.dot(p.origin) << "\n";
            //std::cout << "0: " << 0 << "\n";
            
            Eigen::Vector3d result = coefficients_matrix.colPivHouseholderQr().solve(constraint_vector);
            //std::cout << "result: " << result.transpose() << "\n";

            l.origin = result;

            return l;
        }
    };

	struct line_segment {
		line line_ = line();
		double length = 0;
        line_segment() {}
		line_segment(vector::worldspace origin, vector::worldspace end) {
		  	end -= origin;
		  	length = end.norm();
		  	end = end/end.norm();
		  	line_ = line(origin, end);
		}
        line_segment intersection(line_segment& l) {
            // Assumes that the two line segments are intersecting
            double line0_min = 0;
            double line0_max = length;
            double line1_min = line_.distance_along_line(l.line_.point_along_line(0));
            double line1_max = line_.distance_along_line(l.line_.point_along_line(l.length));
            
            if (line1_min < 0) return l.intersection(*this);

            if (line1_min > line1_max) std::swap(line1_min, line1_max);

            double new_min = fmax(line0_min, line1_min);
            double new_max = fmin(line0_max, line1_max);

            return line_segment(
                line_.point_along_line(new_min),
                line_.point_along_line(new_max)
            );
        }
	};

	struct triangle {
		vector::worldspace points[3];
		triangle() {}
		triangle(vector::worldspace points_[3]) {
		  	points[0] = points_[0];
		  	points[1] = points_[1];
		  	points[2] = points_[2];
		}
		triangle(vector::worldspace p0, vector::worldspace p1, vector::worldspace p2) {
		  	points[0] = p0;
		  	points[1] = p1;
		  	points[2] = p2;
		}
		vector::worldspace get_normal() { // a and b are two vectors representing two sides of the triangle; their cross-product is perpendicular to the triangle
		  	vector::worldspace a = points[1] - points[0];
		  	vector::worldspace b = points[2] - points[0];
		  	return a.cross(b).normalized();
		}
		int point_is_ahead_of_normal(vector::worldspace input_point) {
		  	input_point -= points[0]; // offset the input point to be relative to the triangle
		  	if (input_point.dot(get_normal()) > 0) return 1;
		  	if (input_point.dot(get_normal()) < 0) return -1;
		  	return 0; // if the dot product of the triangle normal and the vector from the triangle's point 0 to the input point is positive, that point is in front of the triangle's plane
		}
		double is_intersecting_ray(ray& r){ // returns distance along ray; taken from https://en.wikipedia.org/wiki/m%c3%b6ller%e2%80%93_trumbore_intersection_algorithm#definitions and modified
		  	constexpr float epsilon = std::numeric_limits<double>::epsilon();

		  	vector::worldspace edge1 = points[1] - points[0];
		  	vector::worldspace edge2 = points[2] - points[0];
		  	vector::worldspace ray_cross_e2 = r.direction.cross(edge2);
		  	double determinant = edge1.dot(ray_cross_e2);

		  	if (determinant > -epsilon && determinant < epsilon) {
				//std::cout << "ray parallel to triangle" << std::endl;
				return -1.0;    // this ray is parallel to this triangle.
		  	}

		  	double inv_determinant = 1.0 / determinant;
		  	vector::worldspace s = r.origin - points[0];
		  	double u = inv_determinant * s.dot(ray_cross_e2);

		  	if (u < 0 || u > 1) {
				//std::cout << "ray fail condition 1" << std::endl;
				return -1.0;
		  	}

		  	vector::worldspace s_cross_e1 = s.cross(edge1);
		  	double v = inv_determinant * r.direction.dot(s_cross_e1);

		  	if (v < 0 || u + v > 1) {
				//std::cout << "ray fail condition 2" << std::endl;
				return -1.0;
		  	}

		  	// at this stage we can compute t to find out where the intersection point is on the line.
		  	double t = inv_determinant * edge2.dot(s_cross_e1);

		  	//if (t > epsilon) // ray intersection
		  	//{
		  	//    //return vec3(r.origin + r.direction * t);
		  	//}
		  	//else // this means that there is a line intersection but not a ray intersection.
		  	//    //return {};
		  	//std::cout << "triangle {" << points[0].str() << ", " << points[1].str() << ", " << points[2].str() << "} intersects ray {" << r.origin.str() << " + " << r.direction.str() << "x} at x = " << t << std::endl;
		  	return t;
		}
		bool is_intersecting_line_segment(line_segment& l) {
		  	if (l.length == 0) return false;
		  	double intersect_distance = is_intersecting_ray(static_cast<ray&>(l.line_));
		  	if (intersect_distance < 0 || intersect_distance > l.length) return false;
		  	return true;
		}
		bool is_intersecting_triangle(triangle& t) {
		  	for (int i = 0; i < 3; i++) {
				line_segment edge = line_segment(t.points[i], t.points[(i+1)%3]);
				if (is_intersecting_line_segment(edge)) {
				  	//std::cout << "triangle {" << points[0].str() << ", " << points[1].str() << ", " << points[2].str() << "} intersecting triangle {" << t.points[0].str() << ", " << t.points[1].str() << ", " << t.points[2].str() << "}" << std::endl;
				  	return true;
				}
		  	}
		  	for (int i = 0; i < 3; i++) {
				line_segment edge = line_segment(points[i], points[(i+1)%3]);
				if (t.is_intersecting_line_segment(edge)) {
				  	//std::cout << "triangle {" << points[0].str() << ", " << points[1].str() << ", " << points[2].str() << "} intersecting triangle {" << t.points[0].str() << ", " << t.points[1].str() << ", " << t.points[2].str() << "}" << std::endl;
				  	return true;
				}
		  	}
		  	return false;
        }
        plane to_plane() {
            return plane(points[0], get_normal());
        }
        std::optional<line_segment> intersection(line& l) {
            // Assumes that the triangles is confirmed to be intersecting the line segment
            std::vector<vector::worldspace> intersection_points;
            for (int i = 0; i < 3; i++) {
                line edge = line(points[i], points[(i+1)%3] - points[i]);
                double t = edge.distance_to_intersection(l);
                if (t < 0) continue;
                if (t > 1) continue;
                intersection_points.push_back(edge.point_along_line(t));
            }
            //std::cout << "[" << globals::error_count << "] intersection(line& l): intersection_points.size() == " << intersection_points.size() << "\n";
            if (intersection_points.size() != 2) {
                globals::error_count++;
                return std::nullopt;
            }
            line_segment output = line_segment(intersection_points[0], intersection_points[1]);
            //std::cout << ":)\n";
            //std::cout << "origin: " << output.line_.origin.transpose() << "\n";
            //std::cout << "direction: " << output.line_.direction.transpose() << "\n";
            //std::cout << "length: " << output.length << "\n";
            return output;
        }
        std::optional<line_segment> intersection(triangle& t) {
            // https://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
            // Assumes that the two triangles are confirmed to be intersecting
            double normal_dot_product = get_normal().dot(t.get_normal());
            if (fabs(fabs(normal_dot_product) - 1) < std::numeric_limits<double>::epsilon()) {
                std::cout << "intersection(triangle& t): triangles are coplanar\n";
                return std::nullopt;
            }
            plane p = t.to_plane();
            std::optional<line> plane_intersection_line = to_plane().line_of_intersection(p);
            if (!plane_intersection_line) {
                std::cout << "intersection(triangle& t): plane_intersection_line returned no value\n";
                return std::nullopt;
            }
            std::optional<line_segment> t0_intersection = intersection(plane_intersection_line.value());
            std::optional<line_segment> t1_intersection = t.intersection(plane_intersection_line.value());
            if (!t0_intersection || !t1_intersection) {
                //std::cout << "intersection(triangle& t): at least one intersection returned no value\n";
                return std::nullopt;
            }
            return t0_intersection.value().intersection(t1_intersection.value());
        }
	};

	struct triangular_prism {
		triangle faces[8]; // base, end, (p0-p1)(base-end) quad tri-pair, (p1-p2)(base-end) quad tri-pair, (p2-p0)(base-end) quad tri-pair
		vector::worldspace central_point;
		triangular_prism() {}
		triangular_prism(triangle& base, vector::worldspace& extrude_direction) {
		  	faces[0] = base;
		  	faces[1] = triangle(
				base.points[0] + extrude_direction,
				base.points[1] + extrude_direction,
				base.points[2] + extrude_direction
		  	);
		  	for (int side = 0; side < 3; side++) {
				faces[side*2+2] = triangle(
				  	base.points[(side+0)%3],
				  	base.points[(side+1)%3],
				  	base.points[(side+0)%3] + extrude_direction
				);
				faces[side*2+3] = triangle(
				  	base.points[(side+1)%3] + extrude_direction,
				  	base.points[(side+1)%3],
				  	base.points[(side+0)%3] + extrude_direction
				);
		  	}
		  	central_point = (base.points[0] + base.points[1] + base.points[2])/3.0 + extrude_direction/2.0; // used later for checking if a point is inside the prism
		}
		bool surrounds_point(vector::worldspace& input_point) {
		  	vector::worldspace a = faces[0].points[1] - faces[0].points[0];
		  	vector::worldspace b = faces[0].points[2] - faces[0].points[0];
		  	double base_area = 0.5*a.cross(b).norm();
		  	vector::worldspace base_normal = faces[0].get_normal();
		  	vector::worldspace extrude_direction = faces[1].points[0] - faces[0].points[0];
		  	double volume = base_area * extrude_direction.dot(base_normal);
		  	if (fabs(volume) < 1e-7) return false;
		  	for (triangle& t : faces) {
				if (t.point_is_ahead_of_normal(central_point) != t.point_is_ahead_of_normal(input_point)) return false; // if the input point is on the same side of all 8 triangles as the central point of the prism, then it is inside the prism
				if (t.point_is_ahead_of_normal(central_point) == 0) return false; // the algorithm breaks in the edgecase where a point is coplanar with a triangle, so ignore the prism in that case 
		  	}
		  	vector::worldspace minimum_coords = faces[0].points[0];
		  	vector::worldspace maximum_coords = faces[0].points[0];
		  	for (triangle& t : faces) {
				for (vector::worldspace& v : t.points) {
				  	minimum_coords.x() = fmin(v.x(), minimum_coords.x());
				  	minimum_coords.y() = fmin(v.y(), minimum_coords.y());
				  	minimum_coords.z() = fmin(v.z(), minimum_coords.z());
				  	maximum_coords.x() = fmax(v.x(), maximum_coords.x());
				  	maximum_coords.y() = fmax(v.y(), maximum_coords.y());
				  	maximum_coords.z() = fmax(v.z(), maximum_coords.z());
				}
		  	}
		  	if (input_point.x() < minimum_coords.x()) return false;
		  	if (input_point.y() < minimum_coords.y()) return false;
		  	if (input_point.z() < minimum_coords.z()) return false;
		  	if (input_point.x() > maximum_coords.x()) return false;
		  	if (input_point.y() > maximum_coords.y()) return false;
		  	if (input_point.z() > maximum_coords.z()) return false;
		  	//std::cout << "point " << input_point.str() << " is inside prism {" << faces[0].points[0].str() << ", " << faces[0].points[1].str() << ", " << faces[0].points[2].str() << "}, {" << faces[1].points[0].str() << ", " << faces[1].points[1].str() << ", " << faces[1].points[2].str() << "} ";
		  	//std::cout << "with center " << central_point.str() << ", ";
		  	//std::cout << "volume = " << volume << ", ";
		  	return true;
		}
    };
}