#ifndef MYRVIZ_H
#define MYRVIZ_H

#include <QWidget>
#include <QString>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <thread>

namespace rviz {
class Display;
class RenderPanel;
class VisualizationManager;
}

// BEGIN_TUTORIAL
// Class "MyViz" implements the top level widget for this example.
class MyViz: public QWidget
{
Q_OBJECT
public:
  MyViz( QWidget* parent = 0 );
  virtual ~MyViz();
  void subCallback(const std_msgs::String& msg);
  void pubThread();

private Q_SLOTS:
  void setThickness( int thickness_percent );
  void setCellSize( int cell_size_percent );
  void setCloudTopic(const QString &newTopic);
  void setCloudSize(int cloudsize);

private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;

  rviz::Display* map_point_;
  rviz::Display* human_point_;

  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  std::thread pub_thread;
};



#endif // MYRVIZ_H
