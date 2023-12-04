#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>


std::pair<double, double> polarToCartesian(double r, double theta);
std::vector<std::vector<double>> clusterRanges(const std::vector<double>& ranges, double th1, int th2); //clustering method


#endif
