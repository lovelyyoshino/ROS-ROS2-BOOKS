#include <iostream>
#include <stdlib.h>

class limit_filter
{
private:
    double valid_number_ = 0;
    double threashold_ = 0.3;

public:
    double filt(double);
    limit_filter(double){};
    ~limit_filter(){};
};

double limit_filter::filt(double a)
{
    if (abs(a - valid_number_) > threashold_)
    {
        return valid_number_;
    }
    valid_number_ = a;
    return a;
}