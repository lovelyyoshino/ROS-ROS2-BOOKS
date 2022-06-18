#include <pluginlib/class_loader.h>
#include <learning_plugin/polygon_base.h>
 
int main(int argc, char** argv)
{
    // 创建一个ClassLoader，用来加载plugin
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("learning_plugin", "polygon_base::RegularPolygon");
 
  try
  {
    // 加载Triangle插件类，路径在polygon_plugins.xml中定义
    boost::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createInstance("polygon_plugins::Triangle");
    // 初始化边长
    triangle->initialize(10.0);
 
    boost::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createInstance("polygon_plugins::Square");
    square->initialize(10.0);
 
    ROS_INFO("Triangle area: %.2f", triangle->area());
    ROS_INFO("Square area: %.2f", square->area());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
 
  return 0;
}