#include "particle_filter.h"

ParticleFilter::ParticleFilter() {
    vmm = new VelocityMotionModel();
    mm = new MeasurementModel();
}

void ParticleFilter::setMap(Eigen::MatrixXf map) {
    grid_map = map;
}

void ParticleFilter::setMeasurementModel(float zmax, float zmin) {
    mm->setMin(zmin);
    mm->setMax(zmax);
}

void ParticleFilter::setParticles(std::vector<Eigen::Vector3f> particles) {
    float weight;
    particle_set.clear();
    M = particles.size();
    weight = 1.0 / particles.size();
    for (const auto particle : particles) {
        particle_set.push_back(std::make_pair(particle, weight));
    }
}

// input: measurement, velocity input, timestep
// output: candidate particles representing robot pose hypotheses
std::vector<std::pair<Eigen::Vector3f, float>> ParticleFilter::filter(std::vector<float> ranges,
std::vector<float> angles, Eigen::Vector3f linear, Eigen::Vector3f angular, float dt)  {
    std::vector<std::pair<Eigen::Vector3f, float>> temp_particle_set;
    std::vector<std::pair<Eigen::Vector3f, float>> new_particle_set;

    Eigen::Vector3f xt;
    std::pair<Eigen::Vector3f, float> prev_particle;
    float wt, normalizer;

    normalizer = 0.0;
    // Generate M new samples given: (particle_set, control, measurements)
    for (int m = 0; m < M; m++) {
        prev_particle = sampleParticle();
        xt = vmm->sampleMotionModel(linear, angular, prev_particle.first, dt);
        while (grid_map(xt(1), xt(0)) == 1) {
            xt = vmm->sampleMotionModel(linear, angular, prev_particle.first, dt);
        }
        // compute the probabilty of the measurement given position and map (landmark positions)
        wt = mm->rangeFinderLikelihood(ranges, angles, xt, grid_map); //(std::vector<float> ranges, std::vector<float> angles, Eigen::Vector3f xt, Eigen::MatrixXf grid_map)
        // update the normalizer variable
        normalizer += wt;
        // update the temporary particle set
        temp_particle_set.push_back(std::make_pair(xt, wt));
    }

    // normalize the new set of particles
    for (int i = 0; i < temp_particle_set.size(); i++) {
        wt = temp_particle_set[i].second;
        temp_particle_set[i].second = wt / normalizer;
    }

    // sample from the new particle set to create a new set of particles
    for (int i = 0; i < temp_particle_set.size(); i++) {
        std::pair<Eigen::Vector3f, float> particle = sampleParticle(temp_particle_set); //temp_particle_set[i];
        new_particle_set.push_back(std::make_pair(particle.first, particle.second));
    }
    // add this new sample set to the global set of particles
    particle_set = new_particle_set;
    return particle_set;
}

std::pair<Eigen::Vector3f, float> ParticleFilter::sampleParticle() {
    int sampled_id;
    std::pair<Eigen::Vector3f, float> sampled_particle;

    std::vector<float> weights;
    for (const auto particle : this->particle_set) {
        weights.push_back(particle.second);
    }

    std::discrete_distribution<int> dd{weights.begin(), weights.end()};
    static std::random_device rd;

    sampled_id = dd(rd);
    sampled_particle = this->particle_set[sampled_id];

    return sampled_particle;
}

std::pair<Eigen::Vector3f, float> ParticleFilter::sampleParticle(std::vector<std::pair<Eigen::Vector3f, float>> curr_particles) {
    int sampled_id;
    std::pair<Eigen::Vector3f, float> sampled_particle;

    std::vector<float> weights;
    for (auto particle : curr_particles) {
        weights.push_back(particle.second);
    }

    std::discrete_distribution<int> dd{weights.begin(), weights.end()};
    static std::random_device rd;

    sampled_id = dd(rd);
    sampled_particle = curr_particles[sampled_id];

    return sampled_particle;
}
