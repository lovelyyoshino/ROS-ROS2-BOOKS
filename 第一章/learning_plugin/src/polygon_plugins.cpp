#include <pluginlib/class_list_macros.h>
#include <learning_plugin/polygon_base.h>
#include <learning_plugin/polygon_plugins.h>
 
//注册插件，宏参数：plugin的实现类，plugin的基类
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)