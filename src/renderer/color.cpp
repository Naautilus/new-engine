#include "color.hpp"

bool color::operator==(const color& c) const {
    if (r != c.r) return false;
    if (g != c.g) return false;
    if (b != c.b) return false;
    return true;
}
bool color::operator<(const color& c) const {
    return r < c.r;
}
color& color::operator+=(const color& c) {
    r += c.r;
    g += c.g;
    b += c.b;
    return *this;
}
color& color::operator/=(const double& d) {
    r /= d;
    g /= d;
    b /= d;
    return *this;
}