#pragma once
#include "model_file_interface.hpp"


/*
Vertices is a list of all the unique vertices that make up a mesh and indices
is a list of all the vertex indices that the triangles in the mesh use, i.e.
the number of triangles is indices.size() / 3.
*/

namespace models {
	template <typename T>
	void write_little_endian(std::ofstream& f, T data) {
		static_assert(sizeof(T) == 4);
		uint32_t raw_bits;
		std::memcpy(&raw_bits, &data, sizeof(T));
		uint8_t bytes[4];
		f << static_cast<int8_t>(raw_bits & 0xFF);
		f << static_cast<int8_t>((raw_bits >> 8) & 0xFF);
		f << static_cast<int8_t>((raw_bits >> 16) & 0xFF);
		f << static_cast<int8_t>((raw_bits >> 24) & 0xFF);
	}
	template <typename T>
	void write_big_endian(std::ofstream& f, T data) {
		static_assert(sizeof(T) == 4);
		uint32_t raw_bits;
		std::memcpy(&raw_bits, &data, sizeof(T));
		uint8_t bytes[4];
		f << static_cast<int8_t>((raw_bits >> 24) & 0xFF);
		f << static_cast<int8_t>((raw_bits >> 16) & 0xFF);
		f << static_cast<int8_t>((raw_bits >> 8) & 0xFF);
		f << static_cast<int8_t>(raw_bits & 0xFF);
	}
	void write_uint32_t(std::ofstream& f, uint32_t data) {
		write_little_endian(f, data);
	}
	void write_float(std::ofstream& f, float data) {
		write_little_endian(f, data);
	}
	void dump_file(std::string filename) {
		std::cout << "--BEGIN DUMP--\n";
		std::ifstream f(filename, std::ios::binary);
		char c[1];
		do {
			f.read((char*)&c, sizeof(c));
			std::cout << std::hex << (int)(c[0]);
		} while (!f.eof());
		std::cout << "--END DUMP--\n" << std::dec;
	}
	mesh stl_to_mesh(std::string model, float r, float g, float b, float sun_factor) {
		std::vector<vertex> vertices;
		std::string filename = MODELS_FILEPATH + model + ".stl";
		std::ifstream f(filename, std::ios::binary);
		if (!f.is_open()) {
            std::cout << "failed to find file " << filename << "\n";
			std::abort();
		}
		{
            char f_output[6];
			f.read((char*)&f_output, 5);
            f_output[5] = '\0';
			if (std::string(f_output) == "solid") {
                std::cout << "file " << filename << " is an ASCII STL, not a binary STL\n";
				std::abort();
			}
		}
		f.ignore(75); // binary stls have 80 bytes of useless header so this skips the rest past the ASCII STL check
		uint32_t triangle_count;
		f.read(reinterpret_cast<char*>(&triangle_count), 4);
		//std::cout << "triangles in file " << filename << ": " << triangle_count << "\n";
		float vertex_[3];
		for (int i = 0; i < triangle_count; i++) {
			f.ignore(12); // surface normal
			//if (f.eof()) std::cout << "EOF reached\n";
			for (int j = 0; j < 3; j++) {
				f.read(reinterpret_cast<char*>(&vertex_), 12);
				vertices.push_back(vertex{
					vertex_[0],
					vertex_[1],
					vertex_[2],
					r, g, b, sun_factor});
			}
			f.ignore(2); // "attribute byte" sometimes used for color in non-standard implementations
			//vertex& v = vertices[3*i];
			//std::cout << i << ": {" << v.x << ", " << v.y << ", " << v.z << "}\t";
			//v = vertices[3*i+1];
			//std::cout << i << ": {" << v.x << ", " << v.y << ", " << v.z << "}\t";
			//v = vertices[3*i+2];
			//std::cout << i << ": {" << v.x << ", " << v.y << ", " << v.z << "}\n";
			
		}
		//std::cout << "vertices:\n";
		//for (int i = 0; i < vertices.size(); i++) {
		//    vertex& v = vertices[i];
		//    std::cout << (i+1) << "/" << vertices.size() << ": {" << v.x << ", " << v.y << ", " << v.z << "}\n";
		//}
		return mesh(vertices);
	}
	void mesh_to_stl(std::string model, mesh mesh_data) {
		std::string filename = MODELS_FILEPATH + model + ".stl";
		std::cout << "writing to file " << filename << "\n";
		std::ofstream f(filename, std::ofstream::out | std::ofstream::trunc | std::ios::binary);
		for (int i = 0; i < 80; i++) f << (uint8_t)0x20;
		size_t index = 0;
		uint32_t triangle_count = mesh_data.indices.size() / 3;
		write_uint32_t(f, triangle_count);
		for (int i = 0; i < triangle_count; i++) {
			//std::cout << "tri " << (i+1) << "/" << triangle_count << "\n";
			write_float(f, 0);
			write_float(f, 0);
			write_float(f, 1);
			for (int j = 0; j < 3; j++) {
				write_float(f, mesh_data.vertices[mesh_data.indices[index]].x);
				write_float(f, mesh_data.vertices[mesh_data.indices[index]].y);
				write_float(f, mesh_data.vertices[mesh_data.indices[index]].z);
				index++;
			}
			for (int j = 0; j < 2; j++) f << (uint8_t)0x0;
		}
	}
}