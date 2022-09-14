#include <iostream>

class first_order_filter
{
private:
    double last_output_ = 0;
    double alpha_ = 0.1;
    bool is_the_first_loop_ = true;

public:
    first_order_filter(){};
    double filt(double);
    ~first_order_filter(){};
};

double first_order_filter::filt(double a)
{
    if (is_the_first_loop_)
    {
        is_the_first_loop_ = false;
        last_output_ = a;
        return a;
    }
    last_output_ = alpha_ * a + (1 - alpha_) * last_output_;
    return last_output_;
}