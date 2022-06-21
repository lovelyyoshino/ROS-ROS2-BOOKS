#ifndef PLUGINLIB_LEARNING__POLYGON_PLUGINS_H_
#define PLUGINLIB_LEARNING__POLYGON_PLUGINS_H_
#include <learning_plugin/polygon_base.h>
#include <cmath>
#define PI acos(-1)
 
namespace polygon_plugins
{
  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      Triangle()
      {
      }
 
      void initialize(double side_length)
      {
        side_length_ = side_length;
      }
 
      double area()
      {
        return (3*side_length_*side_length_)/(4*tan(PI/3));
      }
 
    private:
      double side_length_;
  };
 
  class Square : public polygon_base::RegularPolygon
  {
    public:
      Square()
      {
      }
 
      void initialize(double side_length)
      {
        side_length_ = side_length;
      }
 
      double area()
      {
        return (4*side_length_*side_length_)/(4*tan(PI/4));
      }
 
    private:
      double side_length_;
 
  };
};
#endif