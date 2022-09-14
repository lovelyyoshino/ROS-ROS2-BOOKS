#include <iostream>
#include <Eigen/Dense>
#include <random>
#include "draw2d.hpp"

#define MEASUREMENT_ERROR 0.025

double generate_randown(float n)
{
    std::random_device rd;
    std::default_random_engine rng{rd()};
    std::normal_distribution<double> norm{0, n};
    return norm(rng);
}

class extend_filter_kalman
{
private:
    Eigen::MatrixXd H_lidar;
    Eigen::MatrixXd F_lidar;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

public:
    extend_filter_kalman(double t);
    ~extend_filter_kalman(){};
    void update_ekf(Eigen::Vector2d z, Eigen::Vector4d u, Eigen::Vector4d &x_hat, Eigen::Matrix4d &P_hat);
    Eigen::MatrixXd calculatejacobian(const Eigen::Vector4d &x_state);
    Eigen::Vector4d x_hat_;
    Eigen::Matrix4d P_;
};

extend_filter_kalman::extend_filter_kalman(double t)
{
    H_lidar = Eigen::MatrixXd(2, 4);
    H_lidar << 1, 0, 0, 0,
        0, 1, 0, 0;
    F_lidar = Eigen::MatrixXd(4, 4);
    F_lidar << 1, t, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, t,
        0, 0, 0, 1;
    Q = Eigen::MatrixXd(4, 4);
    double noise = 9;
    Q << pow(t, 4) / 4 * noise, pow(t, 3) / 2 * noise, 0, 0,
        0, 0, pow(t, 4) / 4 * noise, pow(t, 3) / 2 * noise,
        pow(t, 3) / 2 * noise, pow(t, 2) * noise, 0, 0,
        0, 0, pow(t, 3) / 2 * noise, pow(t, 2) * noise;
    R = Eigen::MatrixXd(2, 2);
    R = Eigen::MatrixXd::Identity(2, 2) * MEASUREMENT_ERROR;
}

/**
 * @brief 更新雅可比矩阵
 *
 * @param x_state
 * @return Eigen::MatrixXd 返回雅可比矩阵
 */
Eigen::MatrixXd extend_filter_kalman::calculatejacobian(const Eigen::Vector4d &x_state)
{
    Eigen::MatrixXd jacobian = Eigen::MatrixXd(2, 4);
    double px = x_state(0);
    double py = x_state(2);
    double vx = x_state(1);
    double vy = x_state(3);
    double r = sqrt(px * px + py * py);
    if (pow(px, 2) + pow(py, 2) < 0.00001)
    {
        jacobian << 0, 0, 0, 0,
            0, 0, 0, 0;
    }
    else
    {
        jacobian << px / r, 0, py / r, 0,
            -py / r / r, 0, px / r / r, 0;
    }
    H_lidar = jacobian;
    return jacobian;
}

/**
 * @brief 更新参数
 *
 * @param z 观测量
 * @param u 输入向量
 * @param x_hat 状态量
 * @param P_hat 估计量
 */
void extend_filter_kalman::update_ekf(Eigen::Vector2d z, Eigen::Vector4d u, Eigen::Vector4d &x_hat, Eigen::Matrix4d &P_hat)
{
    // predict
    x_hat = F_lidar * x_hat + u;
    P_hat = F_lidar * P_hat * F_lidar.transpose() + Q;

    // update
    Eigen::Vector2d z_pred;
    z_pred << sqrt(x_hat(0) * x_hat(0) + x_hat(2) * x_hat(2)),
        atan2(x_hat(2), x_hat(0));
    Eigen::MatrixXd S = H_lidar * P_hat * H_lidar.transpose() + R;
    Eigen::MatrixXd K = P_hat * H_lidar.transpose() * S.inverse();
    x_hat = x_hat + K * (z - z_pred);
    x_hat_ = x_hat;
    P_hat = (Eigen::MatrixXd::Identity(4, 4) - K * H_lidar) * P_hat;
    P_ = P_hat;
}

int main(int argc, char const *argv[])
{
    double dt = 0.02;
    extend_filter_kalman *ekf = new extend_filter_kalman(dt);

    // 仿真数据
    Eigen::Vector2d z(0, 0), pos(2, 2), vel(1, 2);

    ekf->calculatejacobian(Eigen::Vector4d(pos(0), vel(0), pos(1), vel(1)));
    Eigen::Matrix4d p_init;
    p_init << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::Vector4d x_init(pos(0), vel(0), pos(1), vel(1)), filtered;

    ekf->update_ekf(Eigen::Vector2d(1, 1), Eigen::Vector4d::Zero(), x_init, p_init);
    filtered = ekf->x_hat_;

    draw2D plot_true("");
    draw2D plot_filter("");
    draw2D plot_noise("");
    draw2D plot_vx("");

    for (double t; t < 10; t += dt)
    {
        pos += vel * dt;
        z << sqrt(pos(0) * pos(0) + pos(1) * pos(1)), atan2(pos(1), pos(0)) + generate_randown(MEASUREMENT_ERROR);

        ekf->calculatejacobian(ekf->x_hat_);
        ekf->update_ekf(z, Eigen::Vector4d::Zero(), ekf->x_hat_, ekf->P_);

        plot_true.set_x(pos(0));
        plot_true.set_y("true", pos(1));

        plot_filter.set_x(ekf->x_hat_[0]);
        plot_filter.set_y("filtered", ekf->x_hat_[2]);

        plot_noise.set_x(z(0) * cos(z(1)));
        plot_noise.set_y("data with noise", z(0) * sin(z(1)));

        plot_vx.set_x(t);
        plot_vx.set_y("vy", vel(1));
        plot_vx.set_y("filtered vy", ekf->x_hat_(3));
    }

    plot_true.draw();
    plot_noise.draw();
    plot_filter.draw();
    plot_vx.draw();

    delete ekf;
}