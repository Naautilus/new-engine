#pragma once
#include "vertex.hpp"
#include "../vector/vector_spaces.hpp"

namespace collision{

struct collider;

}

struct mesh {
	std::vector<vertex> vertices;
	std::vector<size_t> indices;
    mesh();
    mesh(std::vector<vertex>& vertices_);
    mesh(collision::collider& c, float r, float g, float b, float sun);
    mesh subdivide(int count);
};