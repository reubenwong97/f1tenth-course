#include "lab7/geom_helpers.h"

#include <math.h>
#include <algorithm>
#include <tuple>

// Computes indices for grids where laserscan crosses, pass by value since swapping things around
std::vector<std::tuple<int, int>> bresenham(int x0, int y0, int x1, int y1) {
    int dx, dy, error, ystep, y;
    bool is_steep, swapped;
    std::vector<std::tuple<int, int>> points;
    std::tuple<int, int> coord;

    dx = x1 - x0;
    dy = y1 - y0;

    is_steep = abs(dy) > abs(dx);
    // rotate the line
    if (is_steep) {
        x0, y0 = y0, x0;
        x1, y1 = y1, x1;
    }

    swapped = false;
    if (x0 > x1) {
        x0, x1 = x1, x0;
        y0, y1 = y1, y0;
        swapped = true;
    }

    // Recalculate differentials
    dx = x1 - x0;
    dy = y1 - y0;
    
    // calculate error
    error = dx / 2; // int error
    ystep = (y0 < y1) ? 1 : -1;

    // Iterate over bounding box generating points between start and end
    y = y0;
    for (int x=x0; x<x1+1; x++) {
        coord = is_steep ? std::make_tuple(y, x) : std::make_tuple(x, y);
        points.push_back(coord);
        error = error - abs(dy);
        if (error < 0) {
            y = y + ystep;
            error = error + dx;
        }
    }
 
    // Reverse the list if the coordinates were swapped
    if (swapped)
        std::reverse(points.begin(), points.end());
    return points;
}