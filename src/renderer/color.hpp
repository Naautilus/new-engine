#pragma once

struct color{
	float r = 0;
    float g = 0;
    float b = 0;
	bool operator==(const color& c) const;
	bool operator<(const color& c) const;
    color& operator+=(const color& c);
    color& operator/=(const double& d);
	color operator*(const float& f) const;
	color operator+(const float& f) const;
};