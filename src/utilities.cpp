#include "utilities.h"

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

std::vector<Point> polar_to_cartesian(float * ranges)
{
    std::vector<Point> cartesian_points;
    for(int i = 0; i < sizeof(ranges)/sizeof(ranges[0]); i++)
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

/* COMPUTES AVG INNER CLUSTER DISTANCE FOR EVERY CLUSTER
float * avg_cluster_distances(std::vector<Point> ranges, std::vector<Point> centroids)
{
    float avg_clust_dist[centroids.size()];
    int cluster_counter[centroids.size()];
    int j = 0;

    for(int i = 0; i < ranges.size(); i++)
    {
        avg_clust_dist[ranges[i].cluster] += distance(ranges[i], centroids[ranges[i].cluster]);
        cluster_counter[ranges[i].cluster]++;
    }
    
    for(int i = 0; i < centroids.size(); i++)
    {
        avg_clust_dist[i] /= (float)cluster_counter[i];
    }

    return avg_clust_dist;
}
*/

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

std::vector<Point> kmeans(std::vector<Point> ranges, int k)
{
    //Point * centroids = init_centroids(k, 3.5f);
    Point centroids[k];
    init_centroids(centroids, 3.5f);

    float last_avg_distance = 100.0f;
    float curr_avg_distance = 90.0f;
    
    while(abs(last_avg_distance - curr_avg_distance) > 0.1)
    {
        last_avg_distance = curr_avg_distance;
        float min_centroid_distance = 100.0f;
        for(int i = 0; i < ranges.size(); i++)  // This cycle assigns a cluster to every node
        {
            for(int j = 0; j < k; j++)
            {
                if(distance(ranges[i], centroids[j]) < min_centroid_distance)
                {
                    min_centroid_distance = distance(ranges[i], centroids[j]);
                    ranges[i].cluster = centroids[j].cluster;
                }
            }
            min_centroid_distance = 100.0f;
        }

        // Update curr_avg_distance and centroids for next iteration
        curr_avg_distance = avg_cluster_distance(ranges, centroids);
        new_centroids(ranges, centroids);

    }

    return ranges;
}