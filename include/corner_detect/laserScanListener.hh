#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include "filters/filter_chain.h"
#include <math.h>
#include "std_msgs/String.h"

class LaserScanListener{


//public:
  struct PointParamters
  {
    float x;float y;float z;
    int pointType;
    int kf;
    int kb;
    float theta;
    float ci;
  };

  struct topo
  {
    int feat;
    int position;
    std::vector<PointParamters>::iterator beg;
    std::vector<PointParamters>::iterator end;
  };

  struct topo_tree
  {
    std::vector<topo> left;
    std::vector<topo> right;
    std::vector<topo> center;
  };

  struct topo_flag
  {
    bool left_turn_flag;
    bool right_turn_flag;
    bool left_par_flag;
    bool right_par_flag;
    bool adj_flag;
    bool forw_right_flag;
    bool forw_left_flag;
    bool per_flag;
    bool space_left;
    bool space_right;
    bool one_left;
    bool one_right;
  };

  enum GAP_T{FORW,LEFT,RIGHT};
  enum SIDE{QUAD,MULT};
  struct topo_flag flags;
  struct topo_tree robot;
  ros::NodeHandle rn;
  laser_geometry::LaserProjection lprojector;
  tf::TransformListener tf_listener;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
  //sensor_msgs::LaserScan::Ptr scan;
  //sensor_msgs::LaserScan::iterator scan_iter;
  std::vector<float>::iterator scan_ranges_iter;
  std::vector<float>::iterator scan_inten_iter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;
  sensor_msgs::PointCloud2::Ptr cloud_ptr;
  //sensor_msgs::PointCloud2Iterator<sensor_msgs::PointCloud2> cl_it;
  //pcl::CloudIterator<pcl::PointXYZI> pcl_iter;
  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_iter;
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain;
  float theta_min;
  ros::Publisher p_pub;
  visualization_msgs::Marker p;
  std::vector <PointParamters> *p_param;
  std::vector<topo> topo_vec;
  struct topo temp_topo;
  std::vector<PointParamters>::iterator ppit;
  enum features{BREAKPOINT,LINE,CURVE,CORNER};
  enum angle_range{UNKNOWN,acute,obtuse,per,par};
  enum intersec{TI,RI,RT,LI,LT};
  ros::Publisher int_pub;
  int seq_no;
  

  public: LaserScanListener(ros::NodeHandle);
  public: ~LaserScanListener();
  protected: void detectIntersection();
  protected: void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);
  protected: void detectBreakPoint(const sensor_msgs::LaserScan& scan);
  protected: void detectLineSegments(std::vector<PointParamters>::iterator,std::vector<PointParamters>::iterator);
  protected: void detectCurveAndCorner(std::vector<PointParamters>::iterator,std::vector<PointParamters>::iterator);
  protected: float computeEuclidDist(std::vector<PointParamters>::iterator,std::vector<PointParamters>::iterator);
  protected: float computeEuclidDist(pcl::PointCloud<pcl::PointXYZ>::iterator,pcl::PointCloud<pcl::PointXYZ>::iterator);
  protected: void publishPoint(std::vector<PointParamters>::iterator,int);
  protected: void publishPoint(pcl::PointCloud<pcl::PointXYZ>::iterator,int);
  protected: void convertLasertoPoint(std::vector<PointParamters>::iterator,float,float);
  protected: float constrainAngle(float);
  protected: void printTopoFeatures();
  protected: void addTopoFeat(int,int,std::vector<PointParamters>::iterator,std::vector<PointParamters>::iterator);
  protected: void updateFlags(float,struct topo,struct topo,int,int);
  protected: void publishIntersection(int);
};


