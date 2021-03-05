#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include "velocity_motion_model.h"
#include "measurement_model.h"

class ParticleFilter {
private:
    MeasurementModel *mm;
    VelocityMotionModel *vmm;
    Eigen::MatrixXf grid_map;
    int M;
    std::vector<std::pair<Eigen::Vector3f, float>> particle_set;
public:
    ParticleFilter();
    void setMap(Eigen::MatrixXf);
    void setParticles(std::vector<Eigen::Vector3f>);
    std::pair<Eigen::Vector3f, float> sampleParticle();
    void setMeasurementModel(float, float);
    std::pair<Eigen::Vector3f, float> sampleParticle(std::vector<std::pair<Eigen::Vector3f, float>>);
    std::vector<std::pair<Eigen::Vector3f, float>> filter(std::vector<float>, std::vector<float>, Eigen::Vector3f, Eigen::Vector3f, float);
};
#endif

//std::discrete_distribution<int> dd{v.begin(), v.end()}
