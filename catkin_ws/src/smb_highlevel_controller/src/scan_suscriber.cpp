#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"

void scanCallback(const sensor_msgs::LaserScan& msg)
{
  float range_min =msg.ranges[0];
    float n,result;
    n = sizeof (msg.ranges)/sizeof (msg.ranges[0]);
    for(int i=1;i<n;i++){
        if(msg.ranges[i]<range_min){
            result=msg.ranges[i];
        }else{
            result=range_min;
        }
    }
    ROS_INFO_STREAM("The minimum distance is:"<<result);


}

int main(int argc, char* argv[])
{
    ros::init(argc ,argv, "scan_suscriber");

    ros::NodeHandle node_handle;

    ros::Subscriber subscriber=
        node_handle.subscribe("/scan",10, scanCallback);

    ros::spin();

           return 0;
}
