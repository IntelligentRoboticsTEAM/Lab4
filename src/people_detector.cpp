#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include "utils/utils.h"

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // 1) cluster

    // 2) avg value for cluster
    // 2.1) cvt to cartesian

    // 3) publish clusters cartesian coordinates
    
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, callback);

    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
