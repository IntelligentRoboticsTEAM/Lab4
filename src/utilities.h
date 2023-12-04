#ifndef UTILITIES_H
#define UTILITIES_H

#include <vector>

struct Point
{
    float x;
    float y;
    int cluster;
};

Point * kmeans(std::vector<Point> ranges, int k, Point centroids[]);

std::vector<Point> polar_to_cartesian(std::vector<float> ranges);

#endif