#ifndef UTILITIES_H
#define UTILITIES_H

#include <vector>

struct Point
{
    float x;
    float y;
    int cluster;
};

std::vector<Point> kmeans(std::vector<Point> ranges, int k);

std::vector<Point> polar_to_cartesian(float &ranges);

#endif