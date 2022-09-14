#include <iostream>
#include <deque>
#include <algorithm>
using namespace std;

class median_filter
{
private:
    deque<double> q_;
    int size_ = 30;

public:
    median_filter(){};
    double filt(double);
    ~median_filter(){};
};

double median_filter::filt(double a)
{
    q_.push_back(a);
    if (q_.size() > size_)
    {
        q_.pop_front();
    }

    deque<double> temp = q_;
    // 进行排序
    sort(temp.begin(), temp.end());
    return temp[temp.size() / 2];
}