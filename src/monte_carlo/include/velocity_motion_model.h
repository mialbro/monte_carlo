#ifndef VELOCITY_MOTION_MODEL_H
#define VELOCITY_MOTION_MODEL_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <cstdlib>
#include <math.h>


#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include<opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// motion model: p(x_t|x_t-1, u_t)
// measurement model: p(z_t|x_t)
class VelocityMotionModel {
private:
    Eigen::VectorXf alpha;
public:
    VelocityMotionModel();
    Eigen::Vector3f sampleMotionModel(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, float); // (linear, angular, prev_particle.first)
    float sampleND(float);
    float sampleTD(float);
    ~VelocityMotionModel();
};
#endif
