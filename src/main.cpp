#include <QApplication>
#include <ros/ros.h>
#include "../include/ros_qt_demo/myrviz.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "myviz", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  MyViz* myviz = new MyViz();
  myviz->show();

  app.exec();

  delete myviz;
}
