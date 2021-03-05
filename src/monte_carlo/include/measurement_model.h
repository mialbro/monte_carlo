#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H

#include <random>
#include <queue>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

typedef std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<std::pair<float, int>>> MIN_QUEUE;

class MeasurementModel {
private:
    float z_hit, z_rand, z_min, z_max;
public:
    MeasurementModel();
    void setMin(float);
    void setMax(float);
    float normalDistribution(float, float);
    float rangeFinderLikelihood(float, float, Eigen::Vector3f, Eigen::MatrixXf);
    float rangeFinderLikelihood(std::vector<float> ranges, std::vector<float> angles, Eigen::Vector3f xt, Eigen::MatrixXf grid_map);
};
#endif
