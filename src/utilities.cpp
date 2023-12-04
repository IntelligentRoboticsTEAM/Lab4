#include "utilities.h"
#include "ros/ros.h"

#include <vector>
#include <cstdlib>
#include <cmath>
#include <random>


static float distance(Point a, Point b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

static void init_centroids(Point * centroids, float max_range)
{
    srand(2023);

    float min_value = 0.0f;
    float max_value = 3.5f;
    
    for(int i = 0; i < sizeof(centroids)/sizeof(centroids[0]); i++)
    {
        centroids[i] = {
            .x = min_value + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (max_value - min_value))),
            .y = min_value + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (max_value - min_value))),
            .cluster = i
        };
    }
}

std::vector<Point> polar_to_cartesian(std::vector<float> ranges)
{
    std::vector<Point> cartesian_points;
    for(int i = 0; i < ranges.size(); i++)
    {
        if(std::isfinite(ranges[i]))
        {
            Point p = {
                .x = ranges[i] * (float)cos(i*0.008738789707422256),
                .y = ranges[i] * (float)sin(i*0.008738789707422256)
            };
            cartesian_points.push_back(p);
        }
    }

    return cartesian_points;
}

static float avg_cluster_distance(std::vector<Point> ranges, Point centroids[])
{
    float avg_distance = 0.0f;
    for(int i = 0; i < ranges.size(); i++)
    {
        avg_distance += distance(ranges[i], centroids[ranges[i].cluster]);
    }
    
    return avg_distance/(float)ranges.size();
}

static void new_centroids(std::vector<Point> ranges, Point * centroids)
{
    int clust_nodes_counter[sizeof(centroids)/sizeof(centroids[0])];
    for(int i = 0; i < sizeof(centroids)/sizeof(centroids[0]); i++)
    {
        centroids->x = 0;
        centroids->y = 0;
    }

    for(int i = 0; i < ranges.size(); i++)
    {
        centroids[ranges[i].cluster].x += ranges[i].x;
        centroids[ranges[i].cluster].y += ranges[i].y;
        clust_nodes_counter[ranges[i].cluster]++;
    }

    for(int i = 0; i < sizeof(centroids)/sizeof(centroids[0]); i++)
    {
        centroids[i].x /= (float)clust_nodes_counter[i];
        centroids[i].y /= (float)clust_nodes_counter[i];
    }
}

Point * kmeans(std::vector<Point> ranges, int k, Point centroids[])
{
    init_centroids(centroids, 3.5f);

    float last_avg_distance = 100.0f;
    float curr_avg_distance = 90.0f;
    
    while(abs(last_avg_distance - curr_avg_distance) > 0.1)
    {
        last_avg_distance = curr_avg_distance;
        float min_centroid_distance = 100.0f;
        ROS_INFO("entering for i");
        for(int i = 0; i < ranges.size(); i++)  // This cycle assigns a cluster to every node
        {
            ROS_INFO("i: %d", i);
            ROS_INFO("entering for j");
            for(int j = 0; j < 3; j++)
            {
                ROS_INFO("start j: %d", j);
                if(distance(ranges[i], centroids[j]) < min_centroid_distance)
                {
                    min_centroid_distance = distance(ranges[i], centroids[j]);
                    ranges[i].cluster = centroids[j].cluster;
                }
                ROS_INFO("end j: %d", j);
            }
            min_centroid_distance = 100.0f;
        }

        // Update curr_avg_distance and centroids for next iteration
        curr_avg_distance = avg_cluster_distance(ranges, centroids);
        new_centroids(ranges, centroids);

        ROS_INFO("while of kmeans");
    }

    return centroids;
}