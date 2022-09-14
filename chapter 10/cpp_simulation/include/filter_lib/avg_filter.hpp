#include <iostream>
#include <deque>
#include <algorithm>
using namespace std;

class avg_filter
{
private:
    deque<double> q_;
    int size_ = 30;

public:
    avg_filter(){};
    double filt(double);
    ~avg_filter(){};
};

double avg_filter::filt(double a)
{
    q_.push_back(a);
    if (q_.size() > size_)
    {
        q_.pop_front();
    }

    double sum = 0;
    deque<double>::iterator it;
    for (it = q_.begin(); it != q_.end(); it++)
    {
        sum += *it;
    }

    return sum / q_.size();
}