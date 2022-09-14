#include <draw2d.hpp>
#include <limit_filter.hpp>
#include <random>
#include <math.h>

double generate_randown(double a, double mu)
{
    random_device rd;
    default_random_engine rng{rd()};
    normal_distribution<double> norm{a, mu};
    return norm(rng);
}

int main(int argc, char const *argv[])
{
    draw2D _draw_2D("limit filter");
    limit_filter _limit_filter(0);
    double dt = 0.01;
    double len = 10;

    for (double i = 0; i < len; i += dt)
    {
        double true_number = sin(i);
        double number_with_noise = generate_randown(true_number, 0.2);
        double number_filtered = _limit_filter.filt(number_with_noise);

        _draw_2D.set_x(i);
        _draw_2D.set_y("noise", number_with_noise);
        _draw_2D.set_y("filtered", number_filtered);
        _draw_2D.set_y("true", true_number);
    }
    _draw_2D.draw();
    return 0;
}
