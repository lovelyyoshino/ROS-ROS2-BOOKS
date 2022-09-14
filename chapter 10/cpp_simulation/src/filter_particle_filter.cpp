#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

// 可视化
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#include <map>
typedef std::map<std::string, std::string> stringmap;

struct Particle
{

    double x;
    double y;
    double theta;
    double weight;
};

struct LandmarkObs
{
    int id;
    double x;
    double y;
};

class ParticleFilter
{
public:
    ParticleFilter() : num_particles(0), is_initialized(false) {}
    ~ParticleFilter() {}
    void init(double x, double y, double theta, const double std[]);
    void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);
    void updateWeights(const double std_landmark[], std::vector<LandmarkObs> observations,
                       std::vector<LandmarkObs> map_landmarks);
    void resample();
    bool initialized() const
    {
        return is_initialized;
    }
    //每个粒子的所有信息
    std::vector<Particle> particles;

private:
    // 需要的粒子数
    int num_particles;
    // 判断是否初始化
    bool is_initialized;
    // 每个粒子对应的权重
    std::vector<double> weights;
};

// step 1
void ParticleFilter::init(double x, double y, double theta, const double std[])
{
    num_particles = 800; //对粒子数重新赋值
    weights.resize(num_particles);
    particles.resize(num_particles);
    // 传入标准偏差，用于给一个预设的误差值
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);
    std::default_random_engine gen;

    // 创建初始化是厚厚的粒子，并对其进行赋值。此时初始化的信息是随机的参数
    for (int i = 0; i < num_particles; ++i)
    {
        Particle p;
        p.x = dist_x(gen); // 从高斯正态分布中获取一个随机值并更新属性
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1;
        particles[i] = p;
        weights[i] = p.weight;
    }
    std::cout << "initialized" << num_particles << "partcles" << std::endl;
    is_initialized = true;
}

// step 2
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    std::default_random_engine gen;
    for (int i = 0; i < num_particles; ++i)
    {
        Particle *p = &particles[i]; // 获取每个粒子的上一时刻的信息
        // 根据当前的速度和角速度推算出下一时刻的预测位姿
        double new_x = p->x + (velocity / yaw_rate) * (sin(p->theta + yaw_rate * delta_t) - sin(p->theta));
        double new_y = p->y + (velocity / yaw_rate) * (cos(p->theta) - cos(p->theta + yaw_rate * delta_t));
        double new_theta = p->theta + (yaw_rate * delta_t);

        // 给每个测量值加上高斯噪声
        std::normal_distribution<double> dist_x(new_x, std_pos[0]);
        std::normal_distribution<double> dist_y(new_y, std_pos[1]);
        std::normal_distribution<double> dist_theta(new_theta, std_pos[2]);

        // 更新粒子
        p->x = dist_x(gen);
        p->y = dist_y(gen);
        p->theta = dist_theta(gen);
    }
}

// step 3
void ParticleFilter::updateWeights(const double std_landmark[],
                                   std::vector<LandmarkObs> observations, std::vector<LandmarkObs> map_landmarks)
{
    //获取实际的观测所在的位置
    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    double weights_sum = 0;

    for (int i = 0; i < num_particles; ++i)
    {
        Particle *p = &particles[i];
        double wt = 1.0;

        // 将车辆的观测数据转换为地图的坐标系
        for (size_t j = 0; j < observations.size(); ++j)
        {
            // 观测到的点
            LandmarkObs current_obs = observations[j];
            LandmarkObs transformed_obs;

            //转换到地图中
            transformed_obs.x = current_obs.x + p->x;
            transformed_obs.y = current_obs.y + p->y;
            transformed_obs.id = current_obs.id;

            LandmarkObs landmark;
            //设置最近的landmark点
            double distance_min = std::numeric_limits<double>::max();

            for (size_t k = 0; k < map_landmarks.size(); ++k)
            {
                LandmarkObs cur_l = map_landmarks[k];
                double distance = sqrt(pow(transformed_obs.x - cur_l.x, 2) + pow(transformed_obs.y - cur_l.y, 2));
                if (distance < distance_min)
                {
                    distance_min = distance;
                    landmark = cur_l;
                }
            }
            // 使用多元高斯分布更新权重
            double num = exp(-0.5 * (pow((transformed_obs.x - landmark.x), 2) / pow(std_x, 2) + pow((transformed_obs.y - landmark.y), 2) / pow(std_y, 2)));
            double denom = 2 * M_PI * std_x * std_y;
            wt *= num / denom;
        }
        weights_sum += wt;
        //更新权重
        p->weight = wt;
    }
    // 归一化到(0, 1]
    for (int i = 0; i < num_particles; i++)
    {
        Particle *p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }
}

// step 4
void ParticleFilter::resample()
{
    // 取样粒子的置换概率与其重量成正比
    std::default_random_engine gen;

    // [0，n)范围上的随机整数
    //每个整数的概率是它的权重除以所有权重的总和
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    std::vector<Particle> resampled_particles;

    // 随机选择重采样
    for (int i = 0; i < num_particles; i++)
    {
        resampled_particles.push_back(particles[distribution(gen)]);
    }
    particles = resampled_particles;
}

int main()
{
    std::vector<double> best_x, best_y, particle_x, particle_y, true_x, true_y;

    double sigma_pos[3] = {0.3, 0.3, 0.01};
    double sigma_landmark[2] = {0.1, 0.1};
    double pos_array[3] = {3, 3, 0.1};
    std::normal_distribution<double> N_x_init(0, sigma_pos[0]);
    std::normal_distribution<double> N_y_init(0, sigma_pos[1]);
    std::normal_distribution<double> N_theta_init(0, sigma_pos[2]);
    std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
    std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
    ParticleFilter pf;
    std::default_random_engine gen;
    double n_x, n_y, n_theta;
    double delta_t = 1; //间隔时间
    Particle best_particle;
    best_particle.theta = pos_array[2];
    best_particle.x = pos_array[0];
    best_particle.y = pos_array[1];
    for (int m = 0; m < 100; m++)
    {
        if (!pf.initialized())
        {                        // 如果pf初始化还未初始化
            n_x = N_x_init(gen); // 加入噪声
            n_y = N_y_init(gen);
            n_theta = N_theta_init(gen);
            // 第一步，粒子滤波初始化
            pf.init(pos_array[0] + n_x, pos_array[1] + n_y, pos_array[2] + n_theta, sigma_pos);
        }
        else
        {
            // 第二步，预测
            pf.prediction(delta_t, sigma_pos, 0.1, 0.01);
        }
        std::vector<LandmarkObs> observations;
        LandmarkObs mess = {1, 2.0, 2.0};
        observations.push_back(mess);
        mess = {2, 0.0, 0.0};
        observations.push_back(mess);
        mess = {3, 3.0, 3.0};
        observations.push_back(mess);
        mess = {4, 4.0, 4.0};
        observations.push_back(mess);

        std::vector<LandmarkObs> noisy_observations;
        for (size_t j = 0; j < observations.size(); ++j)
        {
            n_x = N_obs_x(gen);
            n_y = N_obs_y(gen);
            LandmarkObs obs = observations[j];
            // this time pose
            double best_particle_x = pos_array[0] + 10 * (sin(pos_array[2] + 0.01) - sin(pos_array[2]));
            double best_particle_y = pos_array[1] + 10 * (cos(pos_array[2]) - cos(pos_array[2] + 0.01));
            obs.id = obs.id;
            obs.x = obs.x - pos_array[0] + n_x; // check
            obs.y = obs.y - pos_array[1] + n_y;
            noisy_observations.push_back(obs);
        }

        // 完成三四步的更新和冲采样
        pf.updateWeights(sigma_landmark, noisy_observations, observations);
        pf.resample();

        //获得结果
        std::vector<Particle> particles = pf.particles;
        int num_particles = static_cast<int>(particles.size());
        double highest_weight = 0.0;
        for (int l = 0; l < num_particles; ++l)
        {
            if (particles[l].weight > highest_weight)
            {
                highest_weight = particles[l].weight;
                best_particle = particles[l];
            }

            // 可视化
            particle_x.push_back(particles[l].x);
            particle_y.push_back(particles[l].y);
        }

        std::cout << pos_array[0] << "," << pos_array[1] << "," << pos_array[2] << "," << best_particle.x << "," << best_particle.y << "," << best_particle.theta << std::endl;
        // 可视化
        true_x.push_back(pos_array[0]);
        true_y.push_back(pos_array[1]);

        // 可视化参数传递
        best_x.push_back(best_particle.x);
        best_y.push_back(best_particle.y);

        pos_array[0] = pos_array[0] + 10 * (sin(pos_array[2] + 0.01) - sin(pos_array[2]));
        pos_array[1] = pos_array[1] + 10 * (cos(pos_array[2]) - cos(pos_array[2] + 0.01));
        pos_array[2] = pos_array[2] + (0.01);
    }

    //可视化
    stringmap property1({{"color", "#cce5cc"}, {"label", "particle"}, {"marker", "."}});
    stringmap property2({{"color", "blue"}, {"label", "best particle"}, {"marker", "*"}});
    stringmap property3({{"color", "red"}, {"label", "true"}, {"marker", "+"}});
    plt::scatter(particle_x, particle_y, 1, property1);
    plt::scatter(best_x, best_y, 10, property2);
    plt::scatter(true_x, true_y, 5, property3);

    plt::title("particle filter");
    plt::legend();
    plt::xlabel("x(m)");
    plt::ylabel("y(m)");
    plt::grid(true);
    plt::show();
}
