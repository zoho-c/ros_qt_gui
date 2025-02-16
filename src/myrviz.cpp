#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QSpinBox>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "../include/mmwave_mappping_by_zhouhao/myrviz.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{
  nh=ros::NodeHandle("~");
  sub=nh.subscribe("/test",1000,&MyViz::subCallback,this);
  pub=nh.advertise<std_msgs::String>("/test_pub",1000);
  pub_thread=std::thread(&MyViz::pubThread,this);
  // Construct and lay out labels and slider controls.
  QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  QLabel* Topic=new QLabel("Topic:");
  QLineEdit* Topic_text=new QLineEdit();
  QLabel* Size=new QLabel("Cloud Size");
  QSpinBox* size=new QSpinBox();
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( thickness_label, 0, 0 );
  controls_layout->addWidget( thickness_slider, 0, 1 );
  controls_layout->addWidget( cell_size_label, 1, 0 );
  controls_layout->addWidget( cell_size_slider, 1, 1 );
  controls_layout->addWidget(Topic,0,4);
  controls_layout->addWidget(Topic_text,0,5);
  controls_layout->addWidget(Size,0,6);
  controls_layout->addWidget(size,0,7);

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );

  // Make signal/slot connections.
  connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
  connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));
  connect(Topic_text,SIGNAL(textChanged(const QString &)),this,SLOT(setCloudTopic(const QString &)));
  connect(size,SIGNAL(valueChanged(int)),this,SLOT(setCloudSize(int)));
  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->setFixedFrame("map");
  manager_->initialize();
  manager_->startUpdate();

  // Create a Grid display.
  grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  ROS_ASSERT( grid_ != NULL );

  map_point_ = manager_->createDisplay( "rviz/PointCloud2", "point cloud", true );
  ROS_ASSERT( map_point_ != NULL );
  map_point_->subProp("Topic")->setValue("/pose_graph/octree");
  map_point_->subProp("Style")->setValue("Points");
  map_point_->subProp("Size (Pixels)")->setValue("2");
  map_point_->subProp("Color Transformer")->setValue("AxisColor");
  map_point_->subProp("Axis")->setValue("Z");
  map_point_->subProp("Invert Rainbow")->setValue("true");
  map_point_->subProp("Decay Time")->setValue("1");

  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Billboards" );
  grid_->subProp( "Color" )->setValue( QColor(Qt::gray ));

  // Initialize the slider values.
  thickness_slider->setValue( 1 );
  cell_size_slider->setValue( 10 );
}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}

// This function is a Qt slot connected to a QSlider's valueChanged()
// signal.  It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
}

void MyViz::setCloudTopic(const QString &newTopic){
  if(map_point_!=NULL){
    map_point_->subProp( "Topic" )->setValue(newTopic);
    //ROS_INFO_STREAM("cloud topic changed to => "<<newTopic.toStdString());
  }
}
void MyViz::setCloudSize(int cloudsize){
  if(map_point_!=NULL){
    map_point_->subProp("Size (Pixels)")->setValue(cloudsize);
  }
}
void MyViz::subCallback(const std_msgs::String& msg){
  ROS_INFO_STREAM("receive message!");
}
void MyViz::pubThread(){
  while(ros::ok()){
    ROS_INFO_STREAM_ONCE("here is in publish process!");
  }
}
