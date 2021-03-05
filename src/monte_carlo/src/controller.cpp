#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <stdlib.h>

void getRandomVelocity(geometry_msgs::Twist &msg, geometry_msgs::TwistStamped &stamped_msg) {
    float x, z;

    x = 4*float(rand())/double(RAND_MAX)-2;
    z = 6*float(rand())/double(RAND_MAX)-3;

    stamped_msg.twist.linear.x = x;
    stamped_msg.twist.angular.z = z;

    msg.linear.x = x;
    msg.angular.z = z;

    stamped_msg.header.stamp = ros::Time::now();
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    // start ros node
    ros::Publisher stamped_pub = nh.advertise<geometry_msgs::TwistStamped>("stamped_cmd_vel", 100);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    srand(time(0)); //Sets up the random number generator
    ros::Rate loop_rate(10); //Sets the loop to publish at a rate of 10Hz
    geometry_msgs::Twist msg;
    geometry_msgs::TwistStamped stamped_msg;

    while(ros::ok()) {
        getRandomVelocity(msg, stamped_msg);
        pub.publish(msg);
        stamped_pub.publish(stamped_msg);
        loop_rate.sleep();
        ros::spinOnce();
    }
    ros::waitForShutdown();
}
