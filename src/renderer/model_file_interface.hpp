#pragma once
#include "../constants/constants.hpp"
#include "../controls/controls.hpp"
#include "vertex.hpp"
#include "mesh.hpp"

extern const std::string MODELS_FILEPATH;

namespace models {
	
mesh stl_to_mesh(std::string model, float r, float g, float b, float sun_factor);
void mesh_to_stl(std::string model, mesh mesh_data);

}