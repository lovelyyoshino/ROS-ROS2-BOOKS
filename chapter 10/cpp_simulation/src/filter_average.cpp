#include "averagefilter.hpp"
#include "matplotlibcpp.h"
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

    Vector3d p, v, a, r, sun_pose;
    p << 0, 0, 0;
    v << 1, 0, 0;
    a << 0, 0, 0;
    r << 0, 0, 0;
    sun_pose << 5, 5, 5;

    double dt = 0.01;
    double k = 5;
    double len = 0;

    draw3D _plot;

    averagefilter _avg;
    _avg._set_number(100);
    Vector3d p_noise;

    for (float i = 0; i < 5; i += dt)
    {
        // this is the true number to be plot
        r = sun_pose - p;
        len = sqrt(r.transpose() * r);
        a = k * r / pow(len, 3);
        v += a * dt;
        p += v * dt;

        // position with noise
        p_noise = p + generate_randown(0.1);
        VectorXd filtered = _avg._filt(p_noise);

        // store the pose to plot
        _plot.input_vector(p, "true number");
        _plot.input_vector(filtered.segment<3>(0), "filtered");
        _plot.input_vector(p_noise, "noise");
    }

    // plot
    _plot.plot();

    return 0;
}