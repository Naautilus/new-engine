#pragma once
#include "../constants/constants.hpp"
#include "../controls/controls.hpp"
#include "vertex.hpp"
#include "mesh.hpp"

const std::string MODELS_FILEPATH = "../models/";

namespace models {
	
template <typename T>
void write_little_endian(std::ofstream& f, T data);
template <typename T>
void write_big_endian(std::ofstream& f, T data);
void write_uint32_t(std::ofstream& f, uint32_t data);
void write_float(std::ofstream& f, float data);
void dump_file(std::string filename);
mesh stl_to_mesh(std::string model, float r, float g, float b, float sun_factor);
void mesh_to_stl(std::string model, mesh mesh_data);

}