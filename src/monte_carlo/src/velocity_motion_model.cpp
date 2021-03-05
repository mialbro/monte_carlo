#include "velocity_motion_model.h"

VelocityMotionModel::VelocityMotionModel() {
    alpha.resize(6);
    alpha << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
};

// p(x_t|u_t,x_t-1)
Eigen::Vector3f VelocityMotionModel::sampleMotionModel(Eigen::Vector3f linear, Eigen::Vector3f angular, Eigen::Vector3f xt_prev, float dt) {
    double v, w, theta, v_hat, w_hat, theta_hat;
    double c_hat, x, y, xp, yp, thetap;
    Eigen::Vector3f xt;

    v = linear(0);
    w = angular(2);
    x = xt_prev(0);
    y = xt_prev(1);
    theta = xt_prev(2);

    // account for noise
    v_hat = v + sampleND(alpha(0)*std::pow(v, 2.0) + alpha(1)*std::pow(w, 2.0));
    w_hat = w + sampleND(alpha(2)*std::pow(v, 2.0) + alpha(3)*std::pow(w, 2.0));
    c_hat = sampleND(alpha(4)*std::pow(v, 2.0) + alpha(5)*std::pow(w, 2.0));

    // update pose with noise
    xp = x - (v_hat / w_hat) * sin(theta) + (v_hat / w_hat) * sin(theta + w_hat*dt);
    yp = y + (v_hat / w_hat) * cos(theta) - (v_hat / w_hat) * cos(theta + w_hat*dt);
    thetap = theta + w_hat*dt + c_hat*dt;

    xt = Eigen::Vector3f(xp, yp, thetap);
    return xt;
}

float VelocityMotionModel::sampleND(float b) {
    float prob, sum = 0;
    for (int i = 0; i < 12; i++) {
        sum += b * (((float) rand() / (RAND_MAX)) * 2 - 1);
    }
    prob = (1.0 / 2.0) * sum;
    return prob;
}


float VelocityMotionModel::sampleTD(float b) {
    float prob;
    int rand_num1, rand_num2;
    rand_num1 = b * (((float) rand() / (RAND_MAX)) * 2 - 1);
    rand_num2 = b * (((float) rand() / (RAND_MAX)) * 2 - 1);
    prob = (std::sqrt(6) / 2) * (rand_num1 + rand_num2);
    return prob;
}
