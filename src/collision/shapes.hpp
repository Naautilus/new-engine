#pragma once
#include "../vector/vector_spaces.hpp"
#include "../globals/globals.hpp"

namespace collision {

struct line {
    extern vector::worldspace origin;
    extern vector::worldspace direction;
    line();
    line(vector::worldspace origin_, vector::worldspace direction_);
    vector::worldspace point_along_line(double t);
    double distance_along_line(vector::worldspace input_point);
    std::optional<double> distance_to_intersection(line& l);
};
struct ray : line {};
struct plane {
    extern vector::worldspace origin;
    extern vector::worldspace direction;
    plane();
    plane(vector::worldspace origin_, vector::worldspace direction_);
    double distance_along_normal(vector::worldspace input_point);
    std::optional<line> line_of_intersection(plane& p);
};
struct line_segment {
    enum line_segment_normalization {
        NORMALIZED,
        UNNORMALIZED
    };
    extern line line_;
    extern double length;
    line_segment() {}
    line_segment(vector::worldspace origin, vector::worldspace end, line_segment_normalization normalization = NORMALIZED);
    line_segment intersection(line_segment& l);
};
struct triangle {
    /*
    it could be useful later to replace every instance of two vectors
    being O(n^2) iterated over with a function like this that can
    optimize it
    
    bool is_intersecting(std::vector<triangle> v0, std::vector<triangle> v1) {}
    */
    vector::worldspace points[3];
    vector::worldspace centroid;
    double radius;
    line_segment edges[3];
    vector::worldspace bounding_box_max;
    vector::worldspace bounding_box_min;
    triangle();
    triangle(vector::worldspace points_[3]);
    triangle(vector::worldspace p0, vector::worldspace p1, vector::worldspace p2);
    
    vector::worldspace get_normal();
    int point_is_ahead_of_normal(vector::worldspace input_point);
    double is_intersecting_ray(ray& r);
    bool is_intersecting_line_segment(line_segment& l);
    bool is_intersecting_triangle(triangle& t);
    bool is_intersecting_triangle_bounding_box(triangle& t);
    plane to_plane();
    std::optional<line_segment> intersection(line& l);
    std::optional<line_segment> intersection(triangle& t);
    void update();
    private:
    void update_centroid();
    void update_radius();
    void update_edges();
    void update_bounding_box();
};
struct triangular_prism {
    std::vector<triangle> faces; // base, end, (p0-p1)(base-end) quad tri-pair, (p1-p2)(base-end) quad tri-pair, (p2-p0)(base-end) quad tri-pair
    vector::worldspace central_point;
    triangular_prism();
    triangular_prism(triangle& base, vector::worldspace& extrude_direction);
    bool surrounds_point(vector::worldspace& input_point);
};

}