#pragma once
#include "../collision/collider.cpp"
#include "vertex.cpp"

struct mesh {
	std::vector<vertex> vertices;
	std::vector<size_t> indices;
	mesh() {
		//std::cout << "New mesh created";
	}
	mesh(std::vector<vertex>& vertices_) {
		std::cout << "New mesh created\n";

		//vertices = vertices_;
		//std::sort(vertices.begin(), vertices.end());
		//vertices.erase(unique(vertices.begin(), vertices.end()), vertices.end());
		//for (vertex& v : vertices_) {
		//    size_t index = std::find(vertices.begin(), vertices.end(), v) - vertices.begin();
		//    indices.push_back(index);
		//}

		vertices = vertices_;
		for (int i = 0; i < vertices.size(); i++) {
			indices.push_back(i);
		}
	}
    mesh(collision::collider& c, float r, float g, float b, float sun) {
        for (collision::triangle& t : c.model_data) {
            for (int i = 0; i < 3; i++) {
                vertices.push_back(vertex{(float)t.points[i].x(), (float)t.points[i].y(), (float)t.points[i].z(), r, g, b, sun});
            }
        }
		for (int i = 0; i < vertices.size(); i++) {
			indices.push_back(i);
		}
    }
};