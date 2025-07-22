#pragma once
#include "../vector/vector_spaces.cpp"

double random(double min, double max) {
	return (rand() / (float)RAND_MAX) * (max - min) + min;
}