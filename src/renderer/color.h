#pragma once

struct color{
	float r = 0;
    float g = 0;
    float b = 0;
	bool operator==(const color& c) const {
		if (r != c.r) return false;
		if (g != c.g) return false;
		if (b != c.b) return false;
		return true;
	}
	bool operator<(const color& c) const {
		return r < c.r;
	}
    color& operator+=(const color& c) {
        r += c.r;
        g += c.g;
        b += c.b;
        return *this;
    }
    color& operator/=(const double& d) {
        r /= d;
        g /= d;
        b /= d;
        return *this;
    }
};