#include "vertex.hpp"

bool vertex::operator==(const vertex& v) const {
    if (x != v.x) return false;
    if (y != v.y) return false;
    if (z != v.z) return false;
    if (r != v.r) return false;
    if (g != v.g) return false;
    if (b != v.b) return false;
    if (sun_factor != v.sun_factor) return false;
    return true;
}
bool vertex::operator<(const vertex& v) const {
    return x < v.x;
}