#include "matplotlibcpp.h"
#include <string>
#include <unordered_map>

namespace plt = matplotlibcpp;
using namespace std;

class draw2D
{
private:
    string _str;
    vector<string> _name;
    vector<double> _x;
    unordered_map<string, vector<double>> _map;

public:
    draw2D(string);
    void set_x(double);
    void set_y(string, double);
    void draw(void);
    ~draw2D();
};

draw2D::draw2D(string str)
{
    plt::title(str);
}

draw2D::~draw2D()
{
}

void draw2D::draw()
{
    for (auto x : _map) //遍历整个map，输出key及其对应的value值
    {
        plt::named_plot(x.first, _x, x.second);
    }
    plt::legend();
    plt::show();
}

void draw2D::set_x(double x)
{
    _x.push_back(x);
}

void draw2D::set_y(string name, double y)
{
    vector<double> vec;
    auto it = _map.find(name);
    if (it == _map.end())
    {
        vec.push_back(y);
        _map.insert({name, vec});
    }
    else
    {
        vec = it->second;
        vec.push_back(y);
        _map.at(name) = vec;
    }
}