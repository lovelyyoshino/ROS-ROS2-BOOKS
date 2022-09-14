#include <iostream>
#include <Eigen/Dense>
#include <vector>
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

class SigmaPoints
{
private:
    double lambda_ = 0;
    double alpha_ = 0;
    double beta_ = 0;

public:
    void computeSigmaPoints(const Eigen::VectorXd mean, const Eigen::MatrixXd cov);
    SigmaPoints(double alpha, double beta, double kappa, int n);
    Eigen::MatrixXd sigma_points_;
    Eigen::VectorXd weights_m_;
    Eigen::VectorXd weights_c_;
    int n_ = 0;
};

/**
 * @brief 初始化sigma point
 *
 * @param alpha
 * @param beta
 * @param kappa
 * @param n
 */
SigmaPoints::SigmaPoints(double alpha, double beta, double kappa, int n)
{
    n_ = n;
    alpha_ = alpha;
    beta_ = beta;
    lambda_ = alpha * alpha * (n_ + kappa) - n_;
    sigma_points_ = Eigen::MatrixXd::Zero(2 * n + 1, n);
    weights_m_ = Eigen::VectorXd::Zero(2 * n + 1);
    weights_c_ = Eigen::VectorXd::Zero(2 * n + 1);
}

/**
 * @brief compute sigma points  生成sigma points
 * @param mean          mean
 * @param cov           covariance
 * @param sigma_points  calculated sigma points
 */
void SigmaPoints::computeSigmaPoints(Eigen::VectorXd mean, Eigen::MatrixXd cov)
{
    Eigen::LLT<Eigen::MatrixXd> llt;
    llt.compute((n_ + lambda_) * cov);
    Eigen::MatrixXd l = llt.matrixL();

    sigma_points_.row(0) = mean;
    for (int i = 0; i < n_; i++)
    {
        sigma_points_.row(1 + i * 2) = mean + l.col(i);
        sigma_points_.row(1 + i * 2 + 1) = mean - l.col(i);
    }

    // 均值的权重
    for (int i = 0; i < 2 * n_ + 1; i++)
    {
        weights_c_[i] = 1 / (2 * (n_ + lambda_));
        weights_m_[i] = weights_c_[i];
    }
    weights_c_[0] = lambda_ / (n_ + lambda_) + 1 - alpha_ * alpha_ + beta_;
    weights_m_[0] = lambda_ / (n_ + lambda_);
}

class unscented_filter_kalman
{
private:
    Eigen::MatrixXd H_lidar;
    Eigen::MatrixXd F_lidar;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

public:
    unscented_filter_kalman(double t);
    ~unscented_filter_kalman(){};
    void update_ukf(Eigen::Vector2d Z, Eigen::Vector4d u, SigmaPoints sigmapoints, Eigen::Matrix4d &P_hat);
    Eigen::Vector4d x_hat_;
    Eigen::Matrix4d P_;
    Eigen::MatrixXd Y_;
    Eigen::Vector4d X_;
};

unscented_filter_kalman::unscented_filter_kalman(double t)
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
 * @brief 更新参数
 *
 * @param z 观测量
 * @param u 输入向量
 * @param x_hat 状态量
 * @param P_hat 估计量
 */
void unscented_filter_kalman::update_ukf(Eigen::Vector2d z, Eigen::Vector4d u, SigmaPoints sigmapoints_x, Eigen::Matrix4d &P_hat)
{
    // 循环次数
    int n_dot = sigmapoints_x.n_ * 2 + 1;

    /****** predict ******/
    // 为了教学，尽管模型是线性关系，我们在这里依然使用关键点求先验估计值以及先验方差
    // 一般来说，这里使用KF预测先验估计以及方差即可
    Y_ = F_lidar * sigmapoints_x.sigma_points_.transpose();
    x_hat_ = Y_ * sigmapoints_x.weights_m_;

    // 计算关键点的加权方差
    Eigen::MatrixXd cov_hat = Eigen::MatrixXd::Zero(P_hat.rows(), P_hat.cols());
    for (int i = 0; i < n_dot; i++)
    {
        Eigen::VectorXd diff = sigmapoints_x.sigma_points_.row(i).transpose() - x_hat_;
        cov_hat += sigmapoints_x.weights_c_(i) * diff * diff.transpose();
    }
    cov_hat += Q;

    /****** update ******/
    // 计算y
    Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(sigmapoints_x.sigma_points_.rows(), z.size());
    Eigen::VectorXd mu_Z = Eigen::VectorXd::Zero(sigmapoints_x.sigma_points_.cols());
    for (int i = 0; i < n_dot; i++)
    {
        double px = 0, py = 0, r = 0, theta = 0;
        px = sigmapoints_x.sigma_points_(i, 0);
        py = sigmapoints_x.sigma_points_(i, 2);
        r = sqrt(px * px + py * py);
        theta = atan2(py, px);
        Z.row(i) << r, theta;
    }
    mu_Z = (sigmapoints_x.weights_m_.transpose() * Z).transpose();
    Eigen::Vector2d y = z - mu_Z;

    // 更新方差
    Eigen::MatrixXd P_z = Eigen::MatrixXd::Zero(z.size(), z.size());
    for (int i = 0; i < n_dot; i++)
    {
        Eigen::VectorXd diff = Z.row(i).transpose() - mu_Z;
        P_z += sigmapoints_x.weights_c_(i) * diff * diff.transpose();
    }
    P_z += R;

    // 更新卡尔曼增益
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(x_hat_.size(), z.size());
    // cout << Z << endl;
    for (int i = 0; i < n_dot; i++)
    {

        Eigen::VectorXd diff_a = Y_.col(i) - x_hat_;
        Eigen::VectorXd diff_b = Z.row(i).transpose() - mu_Z;
        K += sigmapoints_x.weights_c_(i) * diff_a * diff_b.transpose();
    }
    K = K * P_z.inverse();

    X_ = x_hat_ + K * y;
    P_ = P_hat - K * P_z * K.transpose();
}

int main(int argc, char const *argv[])
{
    // 创建无迹卡尔曼滤波器
    double dt = 0.02;
    unscented_filter_kalman *ukf = new unscented_filter_kalman(dt);

    // 创建关键点
    Eigen::Vector4d x_mean;
    Eigen::Matrix4d x_cov;
    x_mean << 2, 0, 2, 0;
    x_cov << 1, 0.1, 0.1, 0.1,
        0.1, 1, 0.1, 0.1,
        0.1, 0.1, 1, 0.1,
        0.1, 0.1, 0.1, 1;
    SigmaPoints x_points(0.1, 2, 1, 4);
    x_points.computeSigmaPoints(x_mean, x_cov);

    // 仿真数据
    Eigen::Vector2d z(0, 0), pos(2, 2), vel(1, 2);
    Eigen::Matrix4d p_init;
    p_init << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Eigen::Vector4d x_init(pos(0), vel(0), pos(1), vel(1)), filtered;

    ukf->update_ukf(z, Eigen::Vector4d::Zero(), x_points, p_init);

    draw2D plot_true("");
    draw2D plot_filter("");
    draw2D plot_noise("");
    draw2D plot_vx("");

    for (double t; t < 10; t += dt)
    {
        pos += vel * dt;
        z << sqrt(pos(0) * pos(0) + pos(1) * pos(1)), atan2(pos(1), pos(0)) + generate_randown(MEASUREMENT_ERROR);

        x_points.computeSigmaPoints(ukf->X_, x_cov);
        ukf->update_ukf(z, Eigen::Vector4d::Zero(), x_points, ukf->P_);

        plot_true.set_x(pos(0));
        plot_true.set_y("true", pos(1));

        plot_filter.set_x(ukf->X_[0]);
        plot_filter.set_y("filtered", ukf->X_[2]);

        plot_noise.set_x(z(0) * cos(z(1)));
        plot_noise.set_y("data with noise", z(0) * sin(z(1)));

        plot_vx.set_x(t);
        plot_vx.set_y("vy", vel(1));
        plot_vx.set_y("filtered vy", ukf->X_(3));
    }

    plot_true.draw();
    plot_noise.draw();
    plot_filter.draw();
    plot_vx.draw();

    delete ukf;
}