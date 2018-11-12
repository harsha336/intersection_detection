#include "corner_detect/laserScanListener.hh"

LaserScanListener::LaserScanListener() :
    nh_private_("~"),
    tf_(),
    scan_mutex_(),
    filter_chain_("sensor_msgs::LaserScan"),
    theta_min_(0.5f),
    scan_recv_(false)
{
    ROS_INFO("LaserScanListener::LaserScanListener: Created new node!");
    nh_private_.param("odom_frame", odom_, std::string("odom"));
    nh_private_.param("base_link_frame", base_link_, std::string("base_link"));
    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    
    laser_sub_.subscribe(nh_,"/scan_filtered",10);
    laser_notifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, base_link_.c_str(), 10);
    laser_notifier_->registerCallback(
      	boost::bind(&LaserScanListener::scanCallback, this, _1));
    laser_notifier_->setTolerance(ros::Duration(0.01));
    filter_chain_.configure("scan_filter_chain");
    int_pub_ = nh_.advertise<corner_detect::MidPoint>("intersection", 0);
}

void LaserScanListener::scanCallback (const sensor_msgs::LaserScan::ConstPtr scan_in)
{
    sensor_msgs::LaserScan scan;
    ros::Publisher output_pub_;

    filter_chain_.update (*scan_in, scan);
	if(scan.ranges.empty()) return;
    if(!convertScanToPointCloud(scan))
    {
	    ROS_WARN_STREAM_THROTTLE(1.0, "LaserScanListener::scanCallback: Error in scan to pcl conversion.");
	    return;
    }
    
	try
    {
    	if(checkLastProcessTime())
		{
    		scan_mutex_.lock();
    		detectBreakPoint(scan);
			scan_mutex_.unlock();
		}
		else
		{
			ROS_INFO_STREAM( "LaserScanListener::scanCallback: Last proc time: "
				   	<< last_proc_time_ << "Allowing time!");
		}
    }
    catch ( std::exception const &e )
    {
    	ROS_INFO_STREAM( "LaserScanListener::scanCallback: Unable to lock: " <<
				e.what());
    }
}

bool LaserScanListener::convertScanToPointCloud(sensor_msgs::LaserScan& scan)
{
  	  ROS_DEBUG_STREAM( "LaserScanListener::convertScanToPointCloud: Method enterd!" );
	  int ign_count = 0;
	  for(unsigned int i = 0;i < scan.ranges.size();i++)
	  {
		  if(std::isinf(scan.ranges[i]))
		  	ign_count++;
	  }
	  sensor_msgs::PointCloud2 cloud;

	  try
	  {
		  if(!buffer_.canTransform(base_link_,scan.header.frame_id,
					  scan.header.stamp + ros::Duration(scan.scan_time),
					  ros::Duration(TRANSFORM_WAIT_TIME)))
		  {
			  ROS_WARN_STREAM( "LaserScanListener::convertScanToPointCloud: Failed to lookup transform from '" <<
					  scan.header.frame_id << "' to '" << base_link_ << "'");
			  return false;
		  }
		  lprojector_.transformLaserScanToPointCloud(base_link_, scan, cloud, buffer_);
	  }
	  catch (const tf::TransformException &e)
	  {
		  ROS_WARN_STREAM_THROTTLE(1.0, "LaserScanListener::convertScanToPointCloud: TF returned a transform exception: " << e.what());
		  return false;
	  }

	  try
	  {
		  pcl::PCLPointCloud2 pcl_pcl2;
		  pcl_conversions::toPCL(cloud, pcl_pcl2);
		  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
                  pcl::fromPCLPointCloud2(pcl_pcl2, pcl_cloud);
		  pcl_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(pcl_cloud);
	  }
	  catch (const pcl::PCLException &e)
	  {
		  ROS_WARN_STREAM_THROTTLE( 1.0, "LaserScanListener::convertScanToPointCloud: Failed to convert a message to pcl type." << e.what());
		  return false;
	  }
	  ROS_DEBUG_STREAM( "LaserScanListener::convertScanToPointCloud: Info regarding point cloud: [" << pcl_cloud_->size() << "]. Ignore count: " << ign_count);
	  return true;
}

void LaserScanListener::detectBreakPoint(const sensor_msgs::LaserScan& scan)
{
    ROS_DEBUG("LaserScanListener::detectBreakPoint: Method entered!");

    float dmax, delta_phi, lambda, sigma_r, prev_inten, euc_dist, prev_range;
    pcl::PointCloud<pcl::PointXYZ>::iterator start, pcl_iter;

    bool prev_range_set = 0;

    delta_phi = scan.angle_increment; lambda = 0.174f; sigma_r = 0.05;

    start = pcl_iter = pcl_cloud_->begin()+1;
	ROS_DEBUG("LaserScanListener::detectBreakPoint: length of pcl cloud: %d", (int)pcl_cloud_->size());

    if(!std::isinf(scan.ranges[0]))
    {
      	prev_range = scan.ranges[0];
      	prev_range_set = 1;
    }
    topo_feat_.clearTopoVector();

    ROS_DEBUG("FOR LOOP BEGIN");
    float angle;
    
	for(int i=1;i<scan.ranges.size();i++){
      	angle = (i*delta_phi) + scan.angle_min;
      	if(std::abs(scan.ranges[i]) > MAX_RANGE_THRESH)
      	{
			prev_range_set = false;
			continue;
      	}

      	if(prev_range_set)
      	{
        	dmax = (prev_range*(sin(delta_phi)/sin(lambda-delta_phi)))+5*sigma_r;
        	euc_dist = computeEuclidDist(pcl_iter, pcl_iter-1);
			ROS_INFO_STREAM("LaserScanListener::detectBreakPoint: Euclidean distance: " <<
                                euc_dist << "dmax: " << dmax );
        	if(euc_dist > dmax)
        	{
	    		ROS_DEBUG_STREAM( "LaserScanListener::detectBreakPoint: Angle reached here: " << angle); 
	    		ROS_DEBUG_STREAM( "LaserScanListener::detectBreakPoint: Points got from laser scan :<" <<scan.ranges[i]*cos(angle) << "," << scan.ranges[i]*sin(angle) << ">");
				int s, e;
				s = std::distance(pcl_cloud_->begin(),start);
				e = std::distance(pcl_cloud_->begin(),pcl_iter-1);
	    		ROS_DEBUG_STREAM( "LaserScanListener::detectBreakPoint: Starting from : " << s << " to :" << e);
	    		detectLineSegments(start, pcl_iter-1);
	  			start = pcl_iter;
        	}
      	}
      	else
      	{
			prev_range = scan.ranges[i];
			prev_range_set = 1;
        	continue;
      	}

      	prev_range = scan.ranges[i];
      	prev_range_set = 1;
      	pcl_iter++;
    }
    
	if(start < (pcl_iter-5))
      	if ( std::abs((start->x)) < 10 )
        	detectLineSegments(start, pcl_iter-1);
    topo_feat_.printTopoFeatures();
    scan_recv_ = true;
    ROS_INFO("FOR LOOP END!");
    return;
}

void LaserScanListener::processScan()
{
	ROS_DEBUG_STREAM( "LaserScanListener::processScan: Method entered!" );
	int rel_ret;
	inter_det::TopoFeature::intersection inter_pose;
	ros::Rate r(10);
	while( nh_.ok())
	{
	    	try
			{
		    	last_proc_time_ = ros::Time::now();
		    	if(scan_recv_)
		    	{
					ROS_INFO("Trying to get mutex!");
		    		scan_mutex_.lock();
					rel_ret = topo_feat_.buildRelation();
					ROS_INFO_STREAM( "LaserScanListener::processScan: Return is: " << rel_ret);
					if( rel_ret > 0 )
					{
						topo_feat_.printRelation();
						inter_pose = topo_feat_.identifyIntersection();
						publishIntersection(inter_pose);
					}
					if(rel_ret != 0)
						topo_feat_.clearTopoFeatures(std::abs(rel_ret));
					scan_mutex_.unlock();
		    	}
			}
			catch( std::exception const &e )
			{
				ROS_INFO_STREAM( "LaserScanListener::processScan: Scan mutex lock failed: " <<
							e.what());
			}
			ros::spinOnce();
			r.sleep();
	}
	if( nh_.ok() != true )
	{
		ROS_ERROR_STREAM( "LaserScanListener::processScan: node handle not ok" );
	}
}

void LaserScanListener::detectLineSegments(pcl::PointCloud<pcl::PointXYZ>::iterator begin, pcl::PointCloud<pcl::PointXYZ>::iterator end)
{
    int inter;
    ROS_DEBUG("LaserScanListener::detectLineSegments:HARSHA: Detecting line segments between two break points <%f,%f> and <%f,%f>.",begin->x,begin->y,end->x,end->y);

    ROS_DEBUG_STREAM( "LaserScanListener::detectLineSegments: Pcl params: " <<
    		      			"begin: " << std::distance(pcl_cloud_->begin(),begin) <<
		      				"end: " << std::distance(pcl_cloud_->begin(),end));

    if(std::distance(begin,end) < 5)
    {
    	ROS_WARN_STREAM( "LaserScanListener::detectLineSegments: The number of scans [" << 
				std::distance(begin,end) << "] is less than the line threshold." );
		return;
    }

    pcl::PointCloud<pcl::PointXYZ>::iterator ppit_ls,ppit_fi,ppit_bi;

    int kf,kb;
    float euc_dist_fi, euc_dist_bi, real_dist_fi, real_dist_bi, dot_product, mag_product, mag_f, mag_b, theta_i, f_vec[2], b_vec[2], Uk;
    bool flag_fi,flag_bi;
    flag_fi = 1;flag_bi = 1;Uk = 0.1f;

    if( begin < pcl_cloud_->begin() && begin > pcl_cloud_->end())
    {
	    ROS_WARN_STREAM( "LaserScanListener:detectLineSegments: Start pointer is NULL!" );
	    return;
    }

    if( end < pcl_cloud_->begin() && end > pcl_cloud_->end() )
    {
	    ROS_WARN_STREAM( "LaserScanListener::detectLineSegments: End pointer null!");
	    return;
    }


    for(ppit_ls = begin+1;ppit_ls<=end-1;ppit_ls++)
    {
		ppit_fi = ppit_ls + 1;
		ppit_bi = ppit_ls - 1;

      	flag_fi = flag_bi = 1;
      	real_dist_fi = real_dist_bi = 0.0f;
      	kf = 0;kb = 0;

      	// STEP 1
      	while(flag_fi)
      	{
        	euc_dist_fi = computeEuclidDist(ppit_fi,ppit_ls);
			real_dist_fi += computeEuclidDist(ppit_fi,ppit_fi-1);
	
			if((euc_dist_fi < (real_dist_fi - Uk)) || (ppit_fi >= end))
			{
	  			flag_fi = 0;
				ROS_DEBUG_STREAM( "Iterator: " << std::distance(pcl_cloud_->begin(),ppit_fi));
	  			ROS_DEBUG_STREAM("LaserScanListener::detectLineSegments:[Forward] Reached the end of pcl_cloud_ or euclidean distance " 
						<< euc_dist_fi << "less than real laser distance " << real_dist_fi);
			}	
			else
				ppit_fi++;
      	}

      	while(flag_bi)
      	{
        	euc_dist_bi = computeEuclidDist(ppit_bi,ppit_ls);
			real_dist_bi += computeEuclidDist(ppit_bi,ppit_bi+1);
			
			if(std::isinf(real_dist_bi))
	  			real_dist_bi = 0.0f;

			if((euc_dist_bi < (real_dist_bi - Uk)) || (ppit_bi <= begin))
			{
	  			flag_bi = 0;
				ROS_DEBUG_STREAM("Iterator: " << std::distance(pcl_cloud_->begin(),ppit_bi));
	  			ROS_DEBUG_STREAM("LaserScanListener::detectLineSegments:[Backward] Reached the beginning of pcl_cloud_ or euclidean distance " << euc_dist_bi << " less than real laser distance" << real_dist_bi);
			}
			else
				ppit_bi--;
      	}

      	//STEP 2
      	f_vec[0] = ppit_ls->x-ppit_fi->x;f_vec[1] = ppit_ls->y-ppit_fi->y;
      	b_vec[0] = ppit_ls->x-ppit_bi->x;b_vec[1] = ppit_ls->y-ppit_bi->y;

      
      	ROS_DEBUG_STREAM( "LaserScanListener::detectLineSegments: Points:\n" << 
		      "Forward : (" << ppit_fi->x << "," << ppit_fi->y << ")\n" <<
		      "Backward: (" << ppit_bi->x << "," << ppit_bi->y << ")\n" <<
		      "Current : (" << ppit_ls->x << "," << ppit_ls->y << ")");

      	dot_product = f_vec[0]*b_vec[0] + f_vec[1]*b_vec[1];
      	mag_f = sqrt(pow(f_vec[0],2)+pow(f_vec[1],2));
      	mag_b = sqrt(pow(b_vec[0],2)+pow(b_vec[1],2));
      	mag_product = mag_f*mag_b;

      	ROS_DEBUG("LaserScanListener::detectLineSegments[Step 2]: Dot product: <%f>, Mag product: <%f>", dot_product,mag_product);
      	theta_i = acos(dot_product/mag_product);
      
      	if(std::isinf(theta_i) || std::isnan(theta_i))
       		theta_i = 0;
	
		ROS_DEBUG_STREAM( "LaserScanListener::detectLineSegments[Step 3]: theta_i is: " << theta_i );
       
      	//STEP 3
      	if(((std::abs(theta_i) <= theta_min_) || (std::abs(theta_i - M_PI) < theta_min_)) && ((real_dist_fi + real_dist_bi)>=LIN_SIGMA))
      	{
			int pos = (std::distance(pcl_cloud_->begin(),ppit_ls-kb) + 
					   std::distance(pcl_cloud_->begin(),ppit_ls+kf))/2;
			if(std::distance(ppit_bi,ppit_fi) > 5)
				topo_feat_.addTopoFeat(LINE, pos, 
					ppit_bi->x, ppit_bi->y, 
					ppit_fi->x,ppit_fi->y, 
					ros::Time::now().toSec());
      	}
    }
    topo_feat_.printTopoFeatures();
    return;
}

float LaserScanListener::constrainAngle(float x)
{
    x = fmod(x + M_PI, 2*M_PI);
    if(x<0)
      	x+=(2*M_PI);
    return (x - M_PI);
  }

  /*void LaserScanListener::detectCurveAndCorner(std::vector<PointParamters>::iterator begin,std::vector<PointParamters>::iterator end)
  {
    //ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Method entered!");
    int ie, ib;
    float Uc, sum_theta, max_theta, ci;
    int type;
    ie = ib = 2;Uc = 0.5;
    std::vector<PointParamters>::iterator temp_ppit_;
    ppit_ = begin+2;
    for(;ppit_<=end-2;ppit_++)
    {
      //ROS_INFO("Point cloud <%d> handling now.",(int)std::distance(p_param_->begin(),ppit_));
      //STEP 4 & 5
      temp_ppit_ = ppit_-ib;

      sum_theta = 0;
      max_theta = temp_ppit_->theta;
      //ROS_INFO("+++++++++++Printing thestas here!+++++++++++++");
      for(;temp_ppit_<=ppit_+ie;temp_ppit_++)
      {
	if(temp_ppit_>p_param_->end() || temp_ppit_<p_param_->begin())
	{
	  ROS_INFO("No more point clouds to handle!");
	  break;
	}
        sum_theta += temp_ppit_->theta;
	//ROS_INFO("Sum theta <%f>, Current theta <%f>",sum_theta,temp_ppit_->theta);
	if(max_theta<temp_ppit_->theta)
	  max_theta = temp_ppit_->theta;
      }
      //ROS_INFO("Max theta <%f>",max_theta);
      //ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++");

      if(!max_theta)
        ci = (sum_theta/5)/max_theta;
      else
        ci = 0.0;
      if (ci>Uc)
      {
        ppit_->pointType = CURVE;
	ppit_->ci = ci;
	//rho = 1/((sum_theta/5)*c);
	//xc = (pcl_iter_-ib)->x + rho*cos(sum_theta);
	//yc = (pcl_iter_-ib)->y + rho*cos(sum_theta);
	topo_feat_.addTopoFeat(CURVE,std::distance(p_param_->begin(),ppit_),(ppit_-ib)->x,(ppit_-ib)->y,(ppit_+ie)->x,(ppit_+ie)->y,ros::Time::now().toSec());
      }
      else
      {
        ROS_INFO("CI <%f> not greater!",ci);
      }

    }
    //ROS_INFO("----------------------AFTER DET CURVES-----------------------------");
    //printTopoFeatures();
    //ROS_INFO("-------------------------------------------------------------------");

    //STEP 6*/
    /*for(pcl_iter_=pcl_cloud_->begin(),ppit_=p_param_->begin();pcl_iter_<pcl_cloud_->end();ppit_++,pcl_iter_++)
    {
      //if(pcl_iter_ == NULL)
      //{
	//ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Null point cloud!");
	//return_;
      //}
      //if(ppit_ == NULL)
      //{
        //ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Null param detected!");
        //return_;
      //}
      if((ppit_->ci>(ppit_+1)->ci) && (ppit_->ci>(ppit_-1)->ci))
      {
        if(ppit_->theta>theta_min_)
	{
	  ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Corn_er detected!");
	  publishPoint(pcl_iter_,CORNER);
	}
      }
    }*/
    //ROS_INFO("LaserScanListener::detectCurveAndCorn_er: Finished!");
  //}

void LaserScanListener::publishPoint(pcl::PointCloud<pcl::PointXYZ>::iterator iter,int point_type)
{
    if(point_type != BREAKPOINT)
    {
      ROS_INFO("not publishing points other than breakpoints currently!");
      return;
    }
    geometry_msgs::Point gp;

    gp.x = iter->x;
    gp.y = iter->y;
    gp.z = iter->z;

    p_.points.push_back(gp);
    ROS_INFO("Publishing the point of type <%d>:<%f,%f,%f>",point_type,gp.x,gp.y,gp.z);
    //p_pub_.publish(p);
}

void LaserScanListener::publishIntersection(inter_det::TopoFeature::intersection i)
{
    corner_detect::MidPoint msg;
    tf::Transform odom_tf;
    tf::Transform ct_pose, pt_pose;
    tf::Pose pp;
    geometry_msgs::Pose p;
    float cd;
    ROS_DEBUG_STREAM("LaserScanListener::publishIntersection: Detected midpoint: " <<
    				i.type << i.p.getOrigin().x() << "," <<
				i.p.getOrigin().y());
    try
    {
      cur_tf_ = buffer_.lookupTransform(odom_, base_link_, ros::Time(0));
      odom_tf = (tf::Transform(tf::Quaternion(cur_tf_.transform.rotation.x,
      					      cur_tf_.transform.rotation.y,
					      cur_tf_.transform.rotation.z,
					      cur_tf_.transform.rotation.w),
			       tf::Vector3(cur_tf_.transform.translation.x,
			       		   cur_tf_.transform.translation.y,
					   cur_tf_.transform.translation.z)));
      if(i.type != UNKW)
      {
    	ct_pose = odom_tf*i.p;
    	tf::poseTFToMsg(ct_pose,p);
    	msg.header.stamp = ros::Time::now();
	msg.child_frame_id = odom_;
	if(!is_.empty())
	{
		
			ROS_DEBUG_STREAM("FINAL: Current Pose: "<<ct_pose.getOrigin().x() <<","
							<<ct_pose.getOrigin().y()<<","
							<<ct_pose.getOrigin().z());
			std::list<std::pair<int,geometry_msgs::Pose>>::iterator it;
			it = is_.begin();
			bool found = false;
			std::pair<int,geometry_msgs::Pose> cur;
			while(it != is_.end())
			{
				cur = *it;
				cd = computeDistance(p,cur.second);
				ROS_DEBUG_STREAM("FINAL: Distance inside while loop between pose-intersection" << cd);
				if(cd < 2)
				{
					msg.reached = "SAME";
					msg.pose = p;
					msg.intersection_name = convertEnumToString(i.type);
					found = true;
					ROS_DEBUG_STREAM("FINAL: =======SAME=======");
					break;
				}
				it++;
			}
			if(!found)
			{
				ROS_DEBUG_STREAM("FINAL: New pose: "<<ct_pose.getOrigin().x() << ","
						<<ct_pose.getOrigin().y() <<","
						<<ct_pose.getOrigin().z());
				is_.emplace_back(std::pair<int,geometry_msgs::Pose>(i.type,p));
				msg.reached = "NEW";
				msg.pose = p;
				msg.intersection_name = convertEnumToString(i.type);
				ROS_DEBUG_STREAM("FINAL: ========NEW=======");
			}
	}
	else 
	{
			ROS_DEBUG_STREAM("FINAL: New pose: "<<ct_pose.getOrigin().x() << ","
						   <<ct_pose.getOrigin().y() <<","
						   <<ct_pose.getOrigin().z());
			msg.reached = "NEW";
			msg.pose = p;
			msg.intersection_name = convertEnumToString(i.type);
			is_.emplace_back(std::pair<int,geometry_msgs::Pose>(i.type,p));
			ROS_DEBUG_STREAM("FINAL: ========NEW=======");
	}
      }
      if(!is_.empty())
      {
		std::list<std::pair<int,geometry_msgs::Pose>>::iterator it;
		for(it = is_.begin(); it != is_.end(); it++)
		{
      			geometry_msgs::Pose op;
			op.position.x = odom_tf.getOrigin().x();
			op.position.y = odom_tf.getOrigin().y();
			op.position.z = odom_tf.getOrigin().z();
			std::pair<int,geometry_msgs::Pose> cur;
			cur = *it;
			float cd = computeDistance(op,cur.second);
			ROS_DEBUG_STREAM("FINAL: No INT but distance between pose-intersection"<<cd);
			if(cd < 0.2)
			{
				msg.reached = "REACHED";
				msg.intersection_name = convertEnumToString(cur.first);
				msg.pose = cur.second;
				ROS_DEBUG_STREAM("FINAL: =====REACHED=====");
				is_.erase(it);
				break;
			}
		}
      }
      std::list<std::pair<int,geometry_msgs::Pose>>::iterator it;
      for(it = is_.begin(); it != is_.end(); it++)
      {
      	std::pair<int,geometry_msgs::Pose> cur;
		cur = *it;
      	ROS_DEBUG_STREAM("FINAL: [" << std::distance(is_.begin(),it) << "]:"
				<< cur.second.position.x << ","
				<< cur.second.position.y << ","
				<< cur.second.position.z);
      }
      ROS_DEBUG_STREAM("================================");
      ROS_DEBUG_STREAM("Odom tf is: " << odom_tf.getOrigin().x() << ","
      				    << odom_tf.getOrigin().y() << ","
				     	<< odom_tf.getOrigin().z());
      
    }
    catch(tf::TransformException &ex)
    {
    	ROS_ERROR("LaserScanListener::detectLineSegments: Clearing topo features : %s",ex.what());
    }
    int_pub_.publish(msg);
}

float LaserScanListener::computeDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  	float dist;
	float dist_x = a.position.x - b.position.x;
	float dist_y = a.position.y - b.position.y;
	dist = sqrt(pow(dist_x,2) + pow(dist_y,2));
	return dist;
}

float LaserScanListener::computeEuclidDist(pcl::PointCloud<pcl::PointXYZ>::iterator pcl1,pcl::PointCloud<pcl::PointXYZ>::iterator pcl2)
{
    float dist;
    float dist_x = pcl1->x - pcl2->x;
    float dist_y = pcl1->y - pcl2->y;

    dist = pow(dist_x,2) + pow(dist_y,2);
    dist = sqrt(dist);
    return dist;
}

LaserScanListener::~LaserScanListener(){

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "scannode");
  ROS_INFO("Node init!");
  LaserScanListener lsListener;
  lsListener.processScan();
  return 0;
}