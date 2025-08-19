#pragma once

struct vertex{
	float x, y, z;
	float r, g, b;
	float sun_factor;
	bool operator==(const vertex& v) const;
	bool operator<(const vertex& v) const;
};