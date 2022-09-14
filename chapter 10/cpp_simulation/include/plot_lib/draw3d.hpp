#include "matplotlibcpp.h"
#include <unordered_map>
#include <string>
#include <Eigen/Dense>

namespace plt = matplotlibcpp;
using namespace Eigen;
using namespace std;

class draw3D
{
private:
    unordered_map<string, vector<double>> _vector_map;
    vector<string> _data_name;

public:
    void input_vector(Vector3d, string);
    void plot(void);
    draw3D();
    ~draw3D();
};

void draw3D::input_vector(Vector3d a, string name)
{
    vector<double> x, y, z;

    auto it = _vector_map.find(name + "_x");
    // 如果没有找到
    if (it == _vector_map.end())
    {
        _data_name.push_back(name);
        // 放入新值
        x.push_back(a[0]);
        _vector_map.insert({name + "_x", x});
    }
    else
    {
        _vector_map[name + "_x"].push_back(a[0]);
    }

    it = _vector_map.find(name + "_y");
    // 如果没有找到
    if (it == _vector_map.end())
    {
        // 放入新值
        y.push_back(a[1]);
        _vector_map.insert({name + "_y", y});
    }
    else
    {
        _vector_map[name + "_y"].push_back(a[1]);
    }

    it = _vector_map.find(name + "_z");
    // 如果没有找到
    if (it == _vector_map.end())
    {
        // 放入新值
        z.push_back(a[2]);
        _vector_map.insert({name + "_z", z});
    }
    else
    {
        _vector_map[name + "_z"].push_back(a[2]);
    }
}

void draw3D::plot()
{
    for (int i = 0; i < _data_name.size(); i++)
    {
        map<string, string> keywords3;
        keywords3.insert(pair<string, string>("label", _data_name[i]));
        plt::plot3(_vector_map[_data_name[i] + "_x"], _vector_map[_data_name[i] + "_y"], _vector_map[_data_name[i] + "_z"], keywords3);
        plt::xlabel("x label");
        plt::ylabel("y label");
        plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
        plt::legend();
    }
    plt::show();
}

draw3D::draw3D()
{
}

draw3D::~draw3D()
{
}