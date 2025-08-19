#pragma once

struct color{
	float r, g, b;
	bool operator==(const color& c) const;
	bool operator<(const color& c) const;
    color& operator+=(const color& c);
    color& operator/=(const double& d);
};