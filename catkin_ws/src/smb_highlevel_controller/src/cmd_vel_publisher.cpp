/* topic:/cmd_vel
 * pub twist msgs to control smb_robot*/
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char* argv[]){

    ros::init(argc, argv, "cmd_vel_publisher");
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_publisher=
            nh.advertise<std_msgs::String>("cmd_vel",1);
    ros::Rate loop_rate(10);

    unsigned int count = 0;
    while (ros::ok()){
        std_msgs::String twist;
        twist.data=""+std::to_string(count);
        ROS_INFO_STREAM(twist.data);
        cmd_vel_publisher.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
        count++;

    }

    
    return 0;
}


