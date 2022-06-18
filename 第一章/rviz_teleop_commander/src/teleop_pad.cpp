#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

#include "teleop_pad.h"

namespace rviz_teleop_commander
{

// 构造函数，初始化变量
TeleopPanel::TeleopPanel( QWidget* parent )
  : rviz::Panel( parent )
, linear_velocity_x( 0 )  
, linear_velocity_y( 0 ) 
, linear_velocity_z( 0 ) 
, angular_velocity_x( 0 )
, angular_velocity_y( 0 )
, angular_velocity_z( 0 )
{
 // 创建一个输入topic命名的窗口
  QVBoxLayout* topic_layout = new QVBoxLayout;
  topic_layout->addWidget( new QLabel( "Teleop Topic:" ));//wen ben kuang 
  output_topic_editor_ = new QLineEdit; // dan hang shu ru kuang
  topic_layout->addWidget( output_topic_editor_ );

  // 创建一个输入线速度的窗口
  topic_layout->addWidget( new QLabel( "Linear Velocity x:" ));
  output_topic_editor_1 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_1 );

  topic_layout->addWidget( new QLabel( "Linear Velocity y:" ));
  output_topic_editor_2 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_2 );

  topic_layout->addWidget( new QLabel( "Linear Velocity z:" ));
  output_topic_editor_3 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_3 );

  // 创建一个输入角速度的窗口
  topic_layout->addWidget( new QLabel( "Angular Velocity x:" ));
  output_topic_editor_4 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_4 );

  topic_layout->addWidget( new QLabel( "Angular Velocity y:" ));
  output_topic_editor_5 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_5 );

  topic_layout->addWidget( new QLabel( "Angular Velocity z:" ));
  output_topic_editor_6 = new QLineEdit;
  topic_layout->addWidget( output_topic_editor_6 );

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addLayout( topic_layout );
  setLayout( layout );

  // 创建一个定时器，用来定时发布消息
  QTimer* output_timer = new QTimer( this );

 // 输入topic命名，回车后，调用updateTopic()
  connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));             
  // 输入线速度值，回车后，调用update_Linear_Velocity()
  connect( output_topic_editor_1, SIGNAL( editingFinished() ), this, SLOT( update_Linear_Velocity() )); 
  connect( output_topic_editor_2, SIGNAL( editingFinished() ), this, SLOT( update_Linear_Velocity() )); 
  connect( output_topic_editor_3, SIGNAL( editingFinished() ), this, SLOT( update_Linear_Velocity() )); 
  // 输入角速度值，回车后，调用update_Angular_Velocity()
  connect( output_topic_editor_4, SIGNAL( editingFinished() ), this, SLOT( update_Angular_Velocity() ));
  connect( output_topic_editor_5, SIGNAL( editingFinished() ), this, SLOT( update_Angular_Velocity() ));
  connect( output_topic_editor_6, SIGNAL( editingFinished() ), this, SLOT( update_Angular_Velocity() ));
  // 设置定时器的回调函数，按周期调用sendVel()
  connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // 设置定时器的周期，100ms
  output_timer->start( 100 );
}

// 更新线速度值
void TeleopPanel::update_Linear_Velocity()
{
    // 获取输入框内的数据
    QString temp_string_1 = output_topic_editor_1->text();
    QString temp_string_2 = output_topic_editor_2->text();
    QString temp_string_3 = output_topic_editor_3->text();
	
	// 将字符串转换成浮点数
    float lin_1 = temp_string_1.toFloat();  
    float lin_2 = temp_string_2.toFloat(); 
    float lin_3 = temp_string_3.toFloat(); 
	// 保存当前的输入值
    linear_velocity_x = lin_1;
    linear_velocity_y = lin_2;
    linear_velocity_z = lin_3;
}

// 更新角速度值
void TeleopPanel::update_Angular_Velocity()
{
    QString temp_string_4 = output_topic_editor_4->text();
    QString temp_string_5 = output_topic_editor_5->text();
    QString temp_string_6 = output_topic_editor_6->text();
    float ang_4 = temp_string_4.toFloat() ;
    float ang_5 = temp_string_5.toFloat() ;
    float ang_6 = temp_string_6.toFloat() ;
    angular_velocity_x = ang_4;
    angular_velocity_y = ang_5;
    angular_velocity_z = ang_6;
}

// 更新topic命名
void TeleopPanel::updateTopic()
{
  setTopic( output_topic_editor_->text() );
}

// 设置topic命名
void TeleopPanel::setTopic( const QString& new_topic )
{
  // 检查topic是否发生改变.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
	
    // 如果命名为空，不发布任何信息
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
	// 否则，初始化publisher
    else
    {
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }

    Q_EMIT configChanged();
  }
}

// 发布消息
void TeleopPanel::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_x;
    msg.linear.y = linear_velocity_y;
    msg.linear.z = linear_velocity_z;
    msg.angular.x = angular_velocity_x;
    msg.angular.y = angular_velocity_y;
    msg.angular.z = angular_velocity_z;
    velocity_publisher_.publish( msg );
  }
}

// 重载父类的功能
void TeleopPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}


// 重载父类的功能，加载配置数据
void TeleopPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    output_topic_editor_->setText( topic );
    updateTopic();
  }
}

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::TeleopPanel,rviz::Panel )
// END_TUTORIAL
