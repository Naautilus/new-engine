#pragma once

struct color{
	extern float r;
    extern float g;
    extern float b;
	bool operator==(const color& c) const;
	bool operator<(const color& c) const;
    color& operator+=(const color& c);
    color& operator/=(const double& d);
};