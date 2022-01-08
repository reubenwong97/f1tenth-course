#include "lab7/geom_helpers.h"

/* DEPRECATED - Working Globally Now
   Input distance in m, angle in rad.
   Because I am looking forward, truncate angles to [-90, 90] degrees.
   Conceptually, index (0, 0), the origin is the position of the car,
   but it is difficult to index the vector array this way, so we will adjust it. 
*/
std::tuple<int, int> toLocalIndex(const double &distance, const double &angle, const double &resolution, const int &width)
{
    std::tuple<int, int> index;
    int xIdx, yIdx;
    double correctedAngle, x, y;

    correctedAngle = -angle;
    x = distance * sin(correctedAngle);
    y = distance * cos(correctedAngle);

    xIdx = floor(x / resolution) + (width / 2);
    yIdx = floor(y / resolution);
    index = std::make_tuple(xIdx, yIdx);

    return index;
}

/* NOTE: Origin is at the bottom right corner of the map
*/
std::tuple<int, int> toGlobalIndex(const double &distance,
                                   const double &angle, const double &top_left_x, const double &top_left_y,
                                   const geometry_msgs::TransformStamped &transformStamped, const nav_msgs::Odometry &pose_msg)
{
    // get position of car in global frame
    double local_pos_x, local_pos_y, global_pos_x, global_pos_y, end_x, end_y;
    int x_idx, y_idx;
    geometry_msgs::PointStamped car_pos;
    local_pos_x = pose_msg.pose.pose.position.x;
    local_pos_y = pose_msg.pose.pose.position.y;
    car_pos = getTransformedPoint(local_pos_x, local_pos_y, transformStamped);
    global_pos_x = car_pos.point.x;
    global_pos_y = car_pos.point.y;

    std::tuple<int, int> index;
    double correctedAngle, x, y;

    correctedAngle = -angle;
    x = distance * sin(correctedAngle);
    y = distance * cos(correctedAngle);

    // get coords of the end points in global frame
    end_x = global_pos_x + x;
    end_y = global_pos_y + y;

    x_idx = floor((end_x - top_left_x) / 2);
    y_idx = floor((end_y - top_left_y) / 2);
    index = std::make_tuple(x_idx, y_idx);
}

geometry_msgs::PointStamped getTransformedPoint(const double &pos_x, const double &pos_y, const geometry_msgs::TransformStamped &transformStamped)
{
    geometry_msgs::PointStamped initialPoint, transformedPoint;
    initialPoint.point.x = pos_x;
    initialPoint.point.y = pos_y;

    tf2::doTransform(initialPoint, transformedPoint, transformStamped);
    return transformedPoint;
}

// Computes indices for grids where laserscan crosses, pass by value since swapping things around
std::vector<std::tuple<int, int>> bresenham(int x0, int y0, int x1, int y1)
{
    int dx, dy, error, ystep, y;
    bool is_steep, swapped;
    std::vector<std::tuple<int, int>> points;
    std::tuple<int, int> coord;

    dx = x1 - x0;
    dy = y1 - y0;

    is_steep = abs(dy) > abs(dx);
    // rotate the line
    if (is_steep)
    {
        x0, y0 = y0, x0;
        x1, y1 = y1, x1;
    }

    swapped = false;
    if (x0 > x1)
    {
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
    for (int x = x0; x < x1 + 1; x++)
    {
        coord = is_steep ? std::make_tuple(y, x) : std::make_tuple(x, y);
        points.push_back(coord);
        error = error - abs(dy);
        if (error < 0)
        {
            y = y + ystep;
            error = error + dx;
        }
    }

    // Reverse the list if the coordinates were swapped
    if (swapped)
        std::reverse(points.begin(), points.end());
    return points;
}