#include <ros/ros.h> 
#include <navigation_test/multi_target.h>

int main(int argc, char** argv) { 
    // ------------------------------------------------------
    ros::init(argc, argv, "navigation_test_node"); 
    ros::Time::init();
    ros::Rate loop_rate(20); // [hz]

    robot_ctrl::NavigationMultiTarget navi_node;

    while(ros::ok())
    {
        navi_node.navigationRun();
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ------------------------------------------------------
    return 0; 
} 