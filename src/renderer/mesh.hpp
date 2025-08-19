#pragma once
#include "../collision/collider.hpp"
#include "vertex.hpp"

struct mesh {
	std::vector<vertex> vertices;
	std::vector<size_t> indices;
    mesh();
    mesh(std::vector<vertex>& vertices_);
    mesh(collision::collider& c, float r, float g, float b, float sun);
};