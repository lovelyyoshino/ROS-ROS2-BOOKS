#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

class kalmanxd
{
private:
    VectorXd X_hat_;   // 最优估计值
    VectorXd B_;       // 输入项
    VectorXd U_;       // 输入矩阵
    MatrixXd Q_;       // 过程噪声
    MatrixXd R_;       // 测量噪声
    MatrixXd F_;       // 转化矩阵
    MatrixXd H_;       // 状态观测矩阵
    MatrixXd P_;       // 协方差
    MatrixXd P_prior_; // 先验协方差
    MatrixXd Kk_;      // 卡尔曼增益
    MatrixXd Zk_;      // 当前测量量
    MatrixXd G_;       // 控制矩阵

    void update_prior_est(MatrixXd);            // 进行先验估计
    void update_p_prior_est(void);              // 进行先验协方差更新
    void update_kalman_gain(void);              // 卡尔曼增益更新
    VectorXd update_best_measurement(VectorXd); // 更新最优测量/后验估计
    void update_p_postterior(void);             // 更新状态估计协方差矩阵P

    // 开关量设置，用于验证数值更新
    bool initialized_ = false;            // 初始化开关量
    bool transfer_matrix_setted_ = false; // 状态转移矩阵
    bool Q_setted_ = false;               // 过程噪声矩阵开关量
    bool P_setted_ = false;               // P矩阵设置开关量
    bool H_setted_ = false;               // 观测矩阵设置
    bool R_setted_ = false;               // R矩阵设置开关量
    bool Zk_setted_ = false;              // 当前测量量
    bool G_setted_ = false;               // 控制矩阵设置开关量
    bool U_setted_ = false;

public:
    VectorXd X_hat_prior_; // 先验值
    bool is_initialized_;  // 初始化判断

    kalmanxd(VectorXd);                                      // 构造函数
    bool initialize(MatrixXd, MatrixXd, MatrixXd, MatrixXd); // 判断是否已经初始化 Q,P,H,R
    bool set_transfer_matrix(MatrixXd);                      // 设置转化矩阵
    bool set_Q(MatrixXd);                                    // 设置过程噪声矩阵
    bool set_R(MatrixXd);                                    // 设置观测噪声矩阵
    bool set_P(MatrixXd);                                    // 设置P矩阵
    bool set_H(MatrixXd);                                    // 设置观测矩阵
    bool set_G(MatrixXd);                                    // 设置控制矩阵
    bool set_U(VectorXd);                                    // 设置输入矩阵

    VectorXd kalman_measure(VectorXd, MatrixXd);
    ~kalmanxd();
};

kalmanxd::kalmanxd(VectorXd X)
{
    X_hat_ = X;
    is_initialized_ = false;
}

/**
 * @brief 进行先验估计
 *
 * @param F 转化矩阵，需要有dt
 */
void kalmanxd::update_prior_est(MatrixXd F)
{
    F_ = F;
    if (G_setted_ == true && U_setted_ == true)
    {
        X_hat_prior_ = F * X_hat_ + G_ * U_;
    }
    else
    {
        X_hat_prior_ = F_ * X_hat_;
    }
}

/**
 * @brief 更新先验估计误差
 *
 */
void kalmanxd::update_p_prior_est(void)
{
    P_prior_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * @brief 更新卡尔曼增益
 *
 */
void kalmanxd::update_kalman_gain(void)
{
    MatrixXd K1 = H_ * P_prior_ * H_.transpose() + R_;
    Kk_ = (P_prior_ * H_.transpose()) * K1.inverse();
}

/**
 * @brief 更新最优估计
 *
 * @param Zk 此次测量值
 * @return MatrixXd 测量最优向量
 */
VectorXd kalmanxd::update_best_measurement(VectorXd Zk)
{
    Zk_ = Zk;
    X_hat_ = X_hat_prior_ + Kk_ * (Zk_ - H_ * X_hat_prior_);
    return X_hat_;
}

/**
 * @brief 更新后验误差
 *
 */
void kalmanxd::update_p_postterior(void)
{
    int i = X_hat_.size();
    MatrixXd I = MatrixXd::Identity(i, i);
    P_ = (I - Kk_ * H_) * P_prior_;
}

/**
 * @brief 进行卡尔曼滤波
 *
 * @param Zk 当前观测值
 * @param F 状态转移矩阵
 * @return VectorXd最优观测结果
 */
VectorXd kalmanxd::kalman_measure(VectorXd Zk, MatrixXd F)
{
    if (initialized_)
    {
        VectorXd best_measurement;
        update_prior_est(F);
        update_p_prior_est();
        update_kalman_gain();
        best_measurement = update_best_measurement(Zk);
        update_p_postterior();
        return best_measurement;
    }
    throw "it didn't initialized!";
    // cout << "it didn't initialized!" << endl;
}

/**
 * @brief 初始化函数
 *
 * @param F 转化矩阵
 * @param Q 过程噪声矩阵
 * @param P 协方差
 * @param H 观测矩阵
 * @param R 观测误差矩阵
 * @param X_hat 初始值
 * @return true
 * @return false
 */
bool kalmanxd::initialize(MatrixXd Q, MatrixXd P, MatrixXd H, MatrixXd R)
{
    //传入初始值
    if (set_Q(Q) && set_P(P) && set_H(H) && set_R(R)) //检测是否更新
    {
        initialized_ = true;
        return true;
    }
    initialized_ = false;
    return false;
}

bool kalmanxd::set_Q(MatrixXd Q)
{
    Q_ = Q;
    Q_setted_ = true;
    return true;
}

bool kalmanxd::set_P(MatrixXd P)
{
    P_ = P;
    P_setted_ = true;
    return true;
}

bool kalmanxd::set_H(MatrixXd H)
{
    H_ = H;
    H_setted_ = true;
    return true;
}

bool kalmanxd::set_R(MatrixXd R)
{
    R_ = R;
    R_setted_ = true;
    return true;
}

bool kalmanxd::set_G(MatrixXd G)
{
    G_ = G;
    G_setted_ = true;
    return true;
}

bool kalmanxd::set_U(VectorXd U)
{
    U_ = U;
    U_setted_ = true;
    return true;
}

kalmanxd::~kalmanxd()
{
}