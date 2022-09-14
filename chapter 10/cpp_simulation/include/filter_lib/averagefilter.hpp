#include <iostream>
#include <Eigen/Dense>
#include <deque>

using namespace std;
using namespace Eigen;

class averagefilter
{
private:
    VectorXd _average(void);
    VectorXd _x_hat;
    deque<VectorXd> _d;
    int _size;
    int _n = 5;

public:
    VectorXd _filt(VectorXd);
    void _set_number(int);
    averagefilter();
    ~averagefilter();
};

averagefilter::averagefilter()
{
}

averagefilter::~averagefilter()
{
}

VectorXd averagefilter::_average(void)
{
    VectorXd sum = VectorXd::Zero(_size);
    for (deque<VectorXd>::const_iterator it = _d.begin(); it != _d.end(); it++)
    {
        sum += *it;
    }
    return sum / _d.size();
}

VectorXd averagefilter::_filt(VectorXd z)
{
    _size = z.size();
    _d.push_back(z);
    if (_d.size() > _n)
    {
        _d.pop_front();
    }
    return _average();
}

void averagefilter::_set_number(int n)
{
    _n = n;
}