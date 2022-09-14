#include "matplotlibcpp.h"
#include "kalman.hpp"
#include "draw3d.hpp"
#include <cmath>
#include <random>

namespace plt = matplotlibcpp;

Vector3d generate_randown(float n)
{
    std::random_device rd;
    std::default_random_engine rng{rd()};
    std::normal_distribution<double> norm{0, n};
    Vector3d v(norm(rng), norm(rng), norm(rng));
    return v;
}

int main(int argc, char const *argv[])
{

    double dt = 0.01;
    double r_noise = 1;
    double q_noise = 0.05;
    // the true number
    Vector3d p, v, a;
    p << 0, 0, 0;
    v << 2, 3, 10;
    a << 0, 0, -9.8;

    // prepare to plot the 3D figure
    draw3D _plot;

    // 位置误差, 这是我们传感器能够观测到的数据
    Vector3d p_noise = p + generate_randown(r_noise);
    Vector3d a_noise = a + generate_randown(r_noise);

    VectorXd x_hat_init = VectorXd::Zero(9);
    kalmanxd _kf(x_hat_init);

    MatrixXd H(6, 9);
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    MatrixXd R = MatrixXd::Identity(6, 6) * r_noise;

    MatrixXd F(9, 9);
    F << 1, 0, 0, dt, 0, 0, 0, 0, 0,
        0, 1, 0, 0, dt, 0, 0, 0, 0,
        0, 0, 1, 0, 0, dt, 0, 0, 0,
        0, 0, 0, 1, 0, 0, dt, 0, 0,
        0, 0, 0, 0, 1, 0, 0, dt, 0,
        0, 0, 0, 0, 0, 1, 0, 0, dt,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    MatrixXd Q = MatrixXd::Identity(9, 9) * q_noise;
    MatrixXd P = MatrixXd::Identity(9, 9);

    _kf.initialize(Q, P, H, R);

    for (float i = 0; i < 5; i += dt)
    {
        // this is the true number to be plot
        v += a * dt;
        p += v * dt;

        // position with noise
        p_noise = p + generate_randown(r_noise);
        a_noise = a + generate_randown(r_noise);

        VectorXd vec(6);
        vec << p_noise, a_noise;
        VectorXd filtered = _kf.kalman_measure(vec, F);

        // 保存位置向量以及噪声
        _plot.input_vector(p, "true number");
        _plot.input_vector(p_noise, "noise");
        // 前三个值表示的是位置，所以采用前三个
        _plot.input_vector(filtered.segment<3>(0), "filtered");
    }

    // plot
    _plot.plot();

    return 0;
}