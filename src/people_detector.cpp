#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>
#include "utils/utils.h"

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    
   
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
