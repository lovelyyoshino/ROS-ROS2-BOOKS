#include <iostream>
#include <deque>
#include <algorithm>
using namespace std;

class avg_med_filter
{
private:
    deque<double> q_;
    int size_ = 30;

public:
    avg_med_filter(){};
    double filt(double);
    ~avg_med_filter(){};
};

double avg_med_filter::filt(double a)
{
    q_.push_back(a);
    if (q_.size() > size_)
    {
        q_.pop_front();
    }

    deque<double> temp = q_;
    sort(temp.begin(), temp.end());

    if (temp.size() >= 3)
    {
        temp.pop_back();
        temp.pop_front();
    }

    double sum = 0;
    deque<double>::iterator it;
    for (it = temp.begin(); it != temp.end(); it++)
    {
        sum += *it;
    }
    cout << sum << endl;

    return sum / temp.size();
}