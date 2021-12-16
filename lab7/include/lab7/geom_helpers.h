#include <vector>
#include <math.h>
#include <algorithm>
#include <tuple>

std::vector<std::tuple<int, int>> bresenham(int x0, int y0, int x1, int y1);
std::tuple<int, int> toIndex(const double &distance, const double &angle, const double &resolution = 0.05, const int &width = 500);