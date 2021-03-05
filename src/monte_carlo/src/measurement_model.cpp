#include "measurement_model.h"
#include "particle_filter.h"

MeasurementModel::MeasurementModel()  {
}

float MeasurementModel::normalDistribution(float mean, float stddev) {
    float sample;
    std::default_random_engine generator;
    std::normal_distribution<float> normal(mean, stddev);
    sample = normal(generator);
    return sample;
}

float MeasurementModel::rangeFinderLikelihood(std::vector<float> ranges, std::vector<float> angles, Eigen::Vector3f xt, Eigen::MatrixXf grid_map) {
    float q, min_dist;
    z_rand = (z_max - z_min) * ((((float) rand()) / (float) z_max)) + z_min;

    q = 1;
    for (int i = 0; i < ranges.size(); i++) {
        min_dist = rangeFinderLikelihood(ranges[i], angles[i], xt, grid_map);
        q = q * (z_hit * normalDistribution(min_dist, 5.0) + (z_rand / z_max));
    }
    return q;
}

float MeasurementModel::rangeFinderLikelihood(float sensor_range, float sensor_angle, Eigen::Vector3f xt, Eigen::MatrixXf grid_map) {
    float q;
    float x, y, theta, z_x, z_y, xk_sensor, yk_sensor, theta_k_sensor;
    float xp, yp, min_dist;

    q = 1;
    x = xt(0);
    y = xt(1);
    theta = xt(2);
    xk_sensor = 0;
    yk_sensor = 0;

    z_x = x + xk_sensor*cos(theta) - yk_sensor*sin(theta) + sensor_range*cos(theta + sensor_angle);
    z_y = y + yk_sensor*cos(theta) + xk_sensor*sin(theta) + sensor_range*sin(theta + sensor_angle);

    MIN_QUEUE min_queue;
    float dx, dy, dist;
    std::vector<Eigen::Vector2i> indices;
    for (int i = 0; i < grid_map.rows(); i++) {
        for (int j = 0; j < grid_map.cols(); j++) {
            if (grid_map(i, j) == 1) {
                dx = z_x - j;
                dy = z_y - i;
                dist = std::sqrt(std::pow(dx, 2.0) + std::pow(dy, 2.0));
                indices.push_back(Eigen::Vector2i(i, j));
                min_queue.push(std::make_pair(dist, indices.size()-1));
            }
        }
    }
    min_dist = min_queue.top().first;
    return min_dist;
}

void MeasurementModel::setMin(float a) {
    z_min = a;
}

void MeasurementModel::setMax(float a) {
    z_max = a;
}
