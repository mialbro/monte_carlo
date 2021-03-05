#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <stdlib.h>
#include <opencv2/core/eigen.hpp>

#include <random>

#include "../include/quadtree.h"
#include "include/particle_filter.h"


ParticleFilter *particle_filter = NULL;

// compute the average distance between points
// this should converge to zero
float computeSTD(std::vector<std::pair<Eigen::Vector3f, float>> particles) {
    float dx, dy, dist, std;
    dist = 0.0;
    std = 0.0;
    for (int i = 0; i < particles.size(); i++) {
        for (int j = 0; j < particles.size(); j++) {
            if (i != j) {
                dx = particles[i].first(0) - particles[j].first(0);
                dy = particles[i].first(1) - particles[j].first(1);
                dist = std::sqrt(std::pow(dx, 2.0) + std::pow(dy, 2.0));
                std += dist;
            }
        }
    }
    std = std / particles.size();
    return std;
}

// sample n particles from the map with random orientations
std::vector<Eigen::Vector3f> sampleMap(std::vector<std::pair<int, int>> free_space, int cnt) {
    int index;
    float x, y, theta;
    std::random_device rd;
    std::mt19937 e2(rd());
    std::vector<Eigen::Vector3f> particle_set;
    for (int i = 0; i < cnt; i++) {
        index = rand() % free_space.size();
        x = free_space[index].first;
        y = free_space[index].second;
        std::uniform_real_distribution<> dist(0, M_PI);
        theta = dist(e2);
        Eigen::Vector3f particle(x, y, theta);
        particle_set.push_back(particle);
    }
    return particle_set;
}

// receive map file
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std_msgs::Header header = msg->header;
    nav_msgs::MapMetaData info = msg->info;
    ROS_INFO("Got map %d %d", info.width, info.height);

    Eigen::MatrixXf grid_map = Eigen::MatrixXf::Ones(info.height, info.width);

    std::vector<std::pair<int, int>> free_space;
    Map map(info.width, info.height);
    for (unsigned int x = 0; x < info.width; x++) {
        for (unsigned int y = 0; y < info.height; y++) {
            int value = Cell(x,y,info.width,msg->data[x+ info.width * y]).value;
            map.Insert(Cell(x,y,info.width,msg->data[x+ info.width * y]));
            if (value == 0) {
                free_space.push_back(std::make_pair(x, y));
                grid_map(y, x) = value;
            }
        }
    }
    std::vector<Eigen::Vector3f> init_particles;
    init_particles = sampleMap(free_space, 20);
    std::cout << particle_filter << std::endl;

    particle_filter = new ParticleFilter();
    particle_filter->setMap(grid_map);
    particle_filter->setParticles(init_particles);
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& vel, const sensor_msgs::LaserScan::ConstPtr& scan) {
    Eigen::Vector3f linear, angular;
    linear = Eigen::Vector3f(vel->twist.linear.x, vel->twist.linear.y, vel->twist.linear.z);
    angular = Eigen::Vector3f(vel->twist.angular.x, vel->twist.angular.y, vel->twist.angular.z);

    float angle, dt;
    std::vector<float> ranges;
    std::vector<float> angles;
    angle = scan->angle_min;
    for (int i = 0; i < ranges.size(); i++) {
        angle = angle + scan->angle_increment;
        if (scan->ranges[i] < scan->range_min && scan->ranges[i] > scan->range_max) {
            ranges.push_back(scan->ranges[i]);
            angles.push_back(angle);
        }
    }

    float std;
    ros::Duration diff;
    std::vector<std::pair<Eigen::Vector3f, float>> new_particles;
    if (particle_filter != NULL) {
        particle_filter->setMeasurementModel(scan->angle_max, scan->angle_min);
        diff = ros::Time::now() - vel->header.stamp;
        dt = diff.toSec();
        new_particles = particle_filter->filter(ranges, angles, linear, angular, dt);
        std = computeSTD(new_particles);
        std::cout << "standard deviation: " << std << std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "monte_carlo");
    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1000, mapCallback);

    srand(time(0)); //Sets up the random number generator
    message_filters::Subscriber<geometry_msgs::TwistStamped> vel_sub(nh, "/stamped_cmd_vel", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, sensor_msgs::LaserScan> sync_pol;

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), vel_sub, scan_sub);
    sync.registerCallback(boost::bind(&velocityCallback, _1, _2));

    // start ros node
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
}
