#include "ros/ros.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf2_ros/transform_listener.h"
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
#include <corner_detect/topoFeatures.hh>
#include <boost/thread/mutex.hpp>

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

  ros::NodeHandle nh_private_, nh_;

  // Variable used but output from this not used at all.
  laser_geometry::LaserProjection lprojector_;
  const int TRANSFORM_WAIT_TIME = 3.0;
  const float PI = 3.14159265358979323846;
  
  // Still to be used!!
  tf::TransformListener tf_;
  geometry_msgs::TransformStamped cur_tf_;

  std::string odom_, base_link_;

  // Subscriber to laser scan
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  //ros::Subscriber laser_sub_;

  //Used to register callback but not for the actual purpose.
  tf::MessageFilter<sensor_msgs::LaserScan> *laser_notifier_;

  // Below not used until now.
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_;
  sensor_msgs::PointCloud2::Ptr cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_iter_;
  filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;
  // ====================================

  // Minimum angle to be considered as a curve
  float theta_min_;

  // Publisher commentde out as of now.
  ros::Publisher p_pub_;

  // Not used at all
  visualization_msgs::Marker p_;

  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  boost::mutex scan_mutex_;

  enum features{BREAKPOINT,LINE,CURVE,CORNER};
  enum angle_range{UNKNOWN,acute,obtuse,per,par};
  enum intersec{TI,RI,RT,LI,LT};
  ros::Publisher int_pub_;
  int seq_no_;
  inter_det::TopoFeature topo_feat_;
  bool scan_recv_;

  ros::Time last_proc_time_;
  

  public: LaserScanListener();
  public: ~LaserScanListener();
  protected: void scanCallback(const sensor_msgs::LaserScan::ConstPtr);
  protected: void detectBreakPoint(const sensor_msgs::LaserScan& scan);
  protected: void detectLineSegments(pcl::PointCloud<pcl::PointXYZ>::iterator,pcl::PointCloud<pcl::PointXYZ>::iterator);
  //protected: void detectCurveAndCorner(pcl::PointCloud<pcl::PointXYZ>::iterator, pcl::PointCloud<pcl::PointXYZ>::iterator);
  protected: float computeEuclidDist(pcl::PointCloud<pcl::PointXYZ>::iterator,pcl::PointCloud<pcl::PointXYZ>::iterator);
  protected: void publishPoint(pcl::PointCloud<pcl::PointXYZ>::iterator,int);
  protected: bool convertScanToPointCloud(sensor_msgs::LaserScan& scan);
  protected: float constrainAngle(float);
  protected: void publishIntersection(int);
  public: void processScan();
  protected: bool checkLastProcessTime()
  		{
			if((ros::Time::now() - last_proc_time_).toSec() < 20)
				return true;
			return false;
		}
};


