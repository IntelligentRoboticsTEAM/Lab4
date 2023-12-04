#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include "utils/utils.h"

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    const std::vector<float>& ranges = msg->ranges;
    const float angle_increment = msg->angle_increment;
    const float angle_min = msg->angle_min;

    // 1) cluster the polar coordinates
    float th1 = 0.3; 
    int th2 = 12;
    std::vector<std::vector<float>> rangeClusters = clusterRanges(ranges, th1, th2);


    // 2)   avg value for cluster
    // 2.1) convert to cartesian coordinates
    std::vector<std::pair<float, float>> avgCartesianCoords;
    computeAvg(rangeClusters, avgCartesianCoords, angle_min, angle_increment);


    // 3) publish clusters cartesian coordinates
    ROS_INFO("Values in the array:");
    for (size_t i = 0; i < avgCartesianCoords.size(); ++i) {
        ROS_INFO("[%zu]: (%f, %f)", i, (float)avgCartesianCoords[i].first, (float)avgCartesianCoords[i].second);
    }
    
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "people_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/scan", 1000, callback);

    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
