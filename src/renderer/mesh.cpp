// top of cpp marker
#include "mesh.hpp"
#include "../collision/collider.hpp"

mesh::mesh() {
    //std::cout << "New mesh created";
}
mesh::mesh(std::vector<vertex>& vertices_) {
    std::cout << "New mesh created\n";

    /*
    currently this fucks up lighting
    vertices = vertices_;
    std::sort(vertices.begin(), vertices.end());
    vertices.erase(unique(vertices.begin(), vertices.end()), vertices.end());
    for (vertex& v : vertices_) {
        size_t index = std::find(vertices.begin(), vertices.end(), v) - vertices.begin();
        indices.push_back(index);
    }
    */

    vertices = vertices_;
    for (int i = 0; i < vertices.size(); i++) {
        indices.push_back(i);
    }
}
mesh::mesh(collision::collider& c, float r, float g, float b, float sun) {
    for (collision::triangle& t : c.model_data) {
        for (int i = 0; i < 3; i++) {
            vertices.push_back(vertex{(float)t.points[i].x(), (float)t.points[i].y(), (float)t.points[i].z(), r, g, b, sun});
        }
    }
    for (int i = 0; i < vertices.size(); i++) {
        indices.push_back(i);
    }
}

namespace {
    std::vector<std::vector<vertex>> _subdivide_triangle_in_half(std::vector<vertex> triangle) {
        std::vector<std::vector<vertex>> output;

        for (int i = 0; i < 3; i++) {
            std::vector<vertex> triangle_;
            triangle_.push_back(triangle[i]);
            triangle_.push_back((triangle[i] + triangle[(i+1)%3]) / 2);
            triangle_.push_back((triangle[i] + triangle[(i+2)%3]) / 2);
            output.push_back(triangle_);
        }

        std::vector<vertex> triangle_;
        triangle_.push_back((triangle[0] + triangle[1]) / 2);
        triangle_.push_back((triangle[1] + triangle[2]) / 2);
        triangle_.push_back((triangle[2] + triangle[0]) / 2);
        output.push_back(triangle_);

        return output;
    }
}

mesh mesh::subdivide(int count) {

    std::cout << "count: " << count << "\n";
    int halving_iterations = 0;
    count /= 2;
    while (count > 0) {
        std::cout << "count: " << count << "\n";
        count /= 2;
        halving_iterations++;
    }
    std::vector<std::vector<vertex>> subdivided_triangles;
    for (int half_iteration = 0; half_iteration < halving_iterations; half_iteration++) {
        for (int i = 0; i < indices.size(); i += 3) {
            auto subdivided_triangles_ = _subdivide_triangle_in_half({vertices[i], vertices[i+1], vertices[i+2]});
            for (auto& triangle : subdivided_triangles_) subdivided_triangles.push_back(triangle);
        }
    }
    std::vector<vertex> vertices;
    for (auto& triangle : subdivided_triangles) {
        for (vertex& v : triangle) {
            v.r = rand() / RAND_MAX;
            v.g = rand() / RAND_MAX;
            v.b = rand() / RAND_MAX;
            vertices.push_back(v);
        }
    }

    return mesh(vertices);
}