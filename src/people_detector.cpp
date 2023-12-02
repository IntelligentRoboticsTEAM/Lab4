#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    double angle = msg->angle_min;
    std::vector<std::pair<double, double>> people_positions;

    for(auto r : msg->ranges)
    {
        if(msg->range_min <= r && r <= msg->range_max)
        {
            // Convert polar coordinates to Cartesian coordinates
            double X = r * cos(angle);
            double Y = r * sin(angle);

            // TODO: Add your logic here to compute the position of each person
        }

        angle += msg->angle_increment;
    }

    // TODO: Publish the people_positions
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_computer");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1000, callback);

    ros::spin();

    return 0;
}
