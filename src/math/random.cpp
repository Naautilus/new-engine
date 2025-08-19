#pragma once
#include "random.hpp"

double random(double min, double max) {
	return (rand() / (float)RAND_MAX) * (max - min) + min;
}