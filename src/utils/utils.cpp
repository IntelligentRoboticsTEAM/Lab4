#include "utils.h"

std::pair<double, double> polarToCartesian(double r, double theta){
    double X = r * cos(theta);
    double Y = r * sin(theta);
    
    return std::make_pair(X, Y);
}


std::vector<std::vector<double>> clusterRanges(const std::vector<double>& ranges, double th1, int th2) 
{
    std::vector<std::vector<double>> clusters;
    std::vector<double> currentCluster;
    int last_index = -1;

    for (int i = 0; i < ranges.size(); i++) {
    if (std::isfinite(ranges[i])) {
        if (currentCluster.empty() || (std::abs(ranges[i] - currentCluster.back()) <= th1 &&
                                    (i - last_index <= th2 || i - last_index <= th2 - ranges.size()))) 
        {
            currentCluster.push_back(ranges[i]);
        } else {
            clusters.push_back(currentCluster); // Add the current cluster to the list of clusters
            currentCluster = { ranges[i] }; // Start a new current cluster
        }

        last_index = i;
    }
}

    // Checks if beginning and end elements belong to same cluster
    if (!currentCluster.empty()) {
        if (std::abs(ranges[0] - currentCluster.back()) <= th1 && (std::abs(0 - last_index <= th2) || std::abs(0 - last_index <= th2 - ranges.size()))) {
            currentCluster.insert(currentCluster.end(), ranges.begin(), ranges.begin() + last_index + 1);
        }
        clusters.push_back(currentCluster); // adding last cluster
    }

    return clusters;
}