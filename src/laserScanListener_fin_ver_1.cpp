#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "corner_detect/laserScanListener.hh"
#include <vector>


  LaserScanListener::LaserScanListener(ros::NodeHandle n) :
    rn(n),
    laser_sub(rn,"/scan_filtered",10),
    laser_notifier(laser_sub,tf_listener,"base_link",10),
    filter_chain("sensor_msgs::LaserScan"),
    pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    theta_min(0.05f)
  {
    ROS_INFO("LaserScanListener::LaserScanListener: Created new node!");
    p_pub = rn.advertise<visualization_msgs::Marker>("interest_point",0);
    laser_notifier.registerCallback(
      boost::bind(&LaserScanListener::scanCallback, this, _1));
    laser_notifier.setTolerance(ros::Duration(0.01));
    filter_chain.configure("scan_filter_chain");
    p.header.frame_id = "base_laser";
    p.header.stamp = ros::Time();
    p.id = 0;
    p.type = visualization_msgs::Marker::POINTS;
    p.action = visualization_msgs::Marker::ADD;
    p.scale.x = 1;
    p.scale.y = 1;
    p.scale.z = 0.1;
    p.color.a = 1.0; // Don't forget to set the alpha!
    p.color.r = 0.0;
    p.color.g = 1.0;
    p.color.b = 0.0;


  }

  void LaserScanListener::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    ROS_INFO("LaserScanListener::scanCallback: Got a laser scan!");
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::LaserScan scan;
    ros::Publisher output_pub_;
    output_pub_ = rn.advertise<sensor_msgs::LaserScan>("output", 1000);
    filter_chain.update (*scan_in, scan);
    ROS_INFO("Publishing filtered scans!");
    output_pub_.publish(scan);

    //this->scan = scan;
    try{
      lprojector.projectLaser(
        scan,cloud);
    }
    catch(tf::TransformException& e)
    {
      ROS_INFO("LaserScanListener::scanCallback: Error converting to point cloud: %s",e.what());
      return;
    }

    ROS_INFO("INFORMATION REGARDING CLOUD FIELDS:");
    ROS_INFO("-----------------------------------");
    ROS_INFO("width : <%d>",(int)cloud.width);
    ROS_INFO("height : <%d>",(int)cloud.height);
    ROS_INFO("Data size: <%d>",(int)cloud.data.size());
    ROS_INFO("Fields information:");
    ROS_INFO("+++++++++++++++++++++++++++++++");
    for (int i=0;i<cloud.fields.size();i++)
    {
      ROS_INFO("Field Name: <%s>",cloud.fields[i].name);
      ROS_INFO("Field datatype : <%d>",(int)cloud.fields[i].datatype);
      ROS_INFO("Field count: <%d>",(int)cloud.fields[i].count);
      ROS_INFO(".........................................");
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*this->pcl_cloud);
ROS_INFO("PREV IMP INFO: cloud size: <%d>,pcl size: <%d>",(int)cloud.data.size(),(int)pcl_cloud->size());
    ROS_INFO("LaserScanListener::scanCallback: PCL cloud is of size <%d>",(int)pcl_cloud->size());
    this->p_param= new std::vector<PointParamters>(scan.ranges.size());
    ROS_INFO("LaserScanListener::scanCallback: Point paramters created with size <%d>",(int)p_param->size());
    detectBreakPoint(scan);
    //detectLineSegments();
    //detectCurveAndCorner();
    p_param->erase(p_param->begin(),p_param->end());
    topo_vec.erase(topo_vec.begin(),topo_vec.end());

  }

  void LaserScanListener::detectBreakPoint(const sensor_msgs::LaserScan& scan)
  {
    ROS_INFO("LaserScanListener::detectBreakPoint: Method entered!");
    float dmax, delta_phi, lambda, sigma_r, prev_inten, euc_dist, prev_range;
    std::vector<PointParamters>::iterator start;
    bool prev_range_set = 0;


    delta_phi = scan.angle_increment; lambda = 0.174f; sigma_r = 0.005;
    ROS_INFO("Angle increment:<%f>",delta_phi);

    //this->pcl_iter = this->pcl_cloud->begin()+1;
    this->ppit = this->p_param->begin();
    if(scan.ranges[0]>10)
    {
      //ppit++;
      start = ppit;
    }
    else
    {
      convertLasertoPoint(ppit,scan.ranges[0],scan.angle_min);
      prev_range = scan.ranges[0];
      prev_range_set = 1;
      start = ppit;
      ppit++;
    }
    ROS_INFO("Size of laser scan data: <%d>",(int)scan.ranges.size());
    ROS_INFO("FOR LOOP BEGIN");
    float angle;
    for(int i=1;i<scan.ranges.size();i++){
      
      angle = (i*delta_phi) + scan.angle_min;
      ROS_INFO("<%d>:Scan data:Range <%f>, Angle <%f>",i,scan.ranges[i],angle);
      if(scan.ranges[i]>10)
      {
        ROS_INFO("Range greater than threshold! so ignoring!");
	ROS_INFO("Making new start:<%d>",(int)std::distance(ppit,p_param->begin()));
	continue;
      }
      convertLasertoPoint(ppit,scan.ranges[i],angle);
      ROS_INFO("<%d>:Converted point data:<%f,%f,%f>,prev_range:<%d>",(int)std::distance(p_param->begin(),ppit),ppit->x,ppit->y,ppit->z,prev_range_set);

      //prev_inten = *--scan_inten_iter;]
      if (!sin(lambda-delta_phi)){
        ROS_INFO("Got ZERO!");
	continue;
      }
      if(prev_range_set)
      {
      dmax = (prev_range*(sin(delta_phi)/sin(lambda-delta_phi)))+3*sigma_r;
      //ROS_INFO("DMAX threshold:<%f>",dmax);
      euc_dist = computeEuclidDist(ppit,ppit-1);
      if(euc_dist>dmax)
      {
        ROS_INFO("LaserScanListener::detectBreakPoint: Break point detected! Scan: <%d>,Euclidean distance: <%f> after <%f,%f> at <%f,%f> with <%d> of scans at angle <%f>",i,euc_dist,start->x,start->y,ppit->x,ppit->y,(int)std::distance(start,ppit),angle);
	ppit->pointType = 1;
	(ppit-1)->pointType = 1;
	publishPoint(ppit,BREAKPOINT);
	publishPoint(ppit-1,BREAKPOINT);
        if (!isinf(start->x))
	{
	  ROS_INFO("Have to call detect line segments with start:<%d> end:<%d>",(int)std::distance(p_param->begin(),start),(int)std::distance(p_param->begin(),ppit-1));
	  detectLineSegments(start,ppit-1);
	  //detectCurveAndCorner(start,ppit-1);
	}
        else
	  ROS_INFO("Starting point is inf~");
	start = ppit;
      }
      else
      {
        ppit->pointType = 0;
      }
      }
      else
      {
        ROS_INFO("Prev range was not set!");
      }
      prev_range = scan.ranges[i];
      prev_range_set = 1;
      ppit++;
      ROS_INFO("Assigning new prev_range:<%f>",prev_range);
    }
    if(start<ppit)
    {
      if (!isinf(start->x))
      {
        ROS_INFO("Have to call detect line segments with start:<%d> end:<%d>",(int)std::distance(p_param->begin(),start),(int)std::distance(p_param->begin(),ppit-1));
        detectLineSegments(start,ppit-1);
        //detectCurveAndCorner(start,ppit-1);
      }
      else
        ROS_INFO("Starting point is inf~");
    }
    else
    {
	ROS_INFO("Should have processed all point clouds!");
    }


    ROS_INFO("Size of topo vec <%d>, Size of ppit <%d>",topo_vec.size(),p_param->size());
    detectIntersection();

    ROS_INFO("FOR LOOP END!");
  }

  void LaserScanListener::convertLasertoPoint(std::vector<PointParamters>::iterator pit,float range, float angle)
  {
    //if(range>10)
      //ROS_INFO("Range greater than 10 : <%f>",range);
    pit->x = range*cos(angle);
    pit->y = range*sin(angle);
    pit->z = 0.0f;
    //if(pit->x>range*cos(0))
      //ROS_INFO("Range greater <%f>",pit->x);

  }
  void LaserScanListener::detectLineSegments(std::vector<PointParamters>::iterator begin, std::vector<PointParamters>::iterator end)
  {
    ROS_INFO("LaserScanListener::detectLineSegments: Method entered!");
    ROS_INFO("LaserScanListener::detectLineSegments:HARSHA: Detecting line segments between two break points <%f,%f> and <%f,%f> having <%d> laser scans.",begin->x,begin->y,end->x,end->y,(int)std::distance(begin,end));

    int size_struct = sizeof(struct topo);

    std::vector<PointParamters>::iterator ppit_ls,ppit_fi,ppit_bi;

    int kf,kb;
    float euc_dist_fi, euc_dist_bi, real_dist_fi, real_dist_bi, dot_product, mag_product, mag_f, mag_b, theta_i, f_vec[2], b_vec[2], Uk;
    bool flag_fi,flag_bi;
    flag_fi = 1;flag_bi = 1;Uk = 1.0f;


    for(ppit_ls = begin+1;ppit_ls<=end-1;ppit_ls++)
    {
      ppit_fi = ppit_bi = ppit_ls;
      flag_fi = flag_bi = 1;
      real_dist_fi = real_dist_bi = 0.0f;
      kf = 0;kb = 0;

      // STEP 1
      while(flag_fi)
      {
        euc_dist_fi = computeEuclidDist(ppit_fi,ppit_ls);

	real_dist_fi += computeEuclidDist(ppit_fi,ppit_fi-1);
	if(isinf(real_dist_fi))
	  real_dist_fi = 0.0f;


	if((euc_dist_fi < (real_dist_fi - Uk)) || (ppit_fi > end))
	{
	  flag_fi = 0;
	  ROS_INFO("LaserScanListener::detectLineSegments: Reached the end of pcl_cloud or euclidean distance <%f> less than real laser distance <%f>",euc_dist_fi,real_dist_fi);
	}
        if(euc_dist_fi < (real_dist_fi - Uk))
        {
          ROS_INFO("IMP: LINE ENDED euc dist<%f>,Real dist<%f>,FORWARD",euc_dist_fi,real_dist_fi);
        }
	ppit_fi++;kf++;
	if(!flag_fi)
	{
	if (ppit_fi >= end)
	  ROS_INFO("LaserScanListener::detectLineSegments: Reached the end!kf <%d>",kf);
	else
	  ROS_INFO("FI loop:DIstnace <%f> is less than the threshold <%f>:kf <%d>",euc_dist_fi,(real_dist_fi - Uk),kf);
	}
      }

      while(flag_bi)
      {
        euc_dist_bi = computeEuclidDist(ppit_bi,ppit_ls);

	real_dist_bi += computeEuclidDist(ppit_bi,ppit_bi-1);
	if(isinf(real_dist_bi))
	  real_dist_bi = 0.0f;

	if((euc_dist_bi < (real_dist_bi - Uk)) || (ppit_bi < begin))
	{
	  flag_bi = 0;
	  ROS_INFO("LaserScanListener::detectLineSegments: Reached the beginning of pcl_cloud or euclidean distance <%f> greater than real laser distance <%f>",euc_dist_bi,real_dist_bi);
	}
	if(euc_dist_bi < (real_dist_bi - Uk))
	{
	  ROS_INFO("IMP: LINE ENDED euc dist<%f>,Real dist<%f> BACKWARD",euc_dist_bi,real_dist_bi);
	}

	ppit_bi--;kb++;
	if(!flag_bi)
	{
	if (ppit_bi < begin)
	  ROS_INFO("LaserScanListener::detectLineSegments: Reached the beginning!kb <%d>",kb);
	else
	  ROS_INFO("BI loop:DIstnace <%f> is less than the threshold <%f>:kb <%d>",euc_dist_bi,(real_dist_bi - Uk),kb);
	}
      }

	kf-=2;kb-=2;
      ROS_INFO("Printing kf and kb:<%d,%d>",kf,kb);

      ppit_ls->kf = kf;ppit_ls->kb = kb;
      ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      ROS_INFO("Forward vec points:<%f,%f> Backward vec points:<%f,%f> Current point:<%f,%f>",(ppit_ls+kf)->x,(ppit_ls+kf)->y,(ppit_ls-kb)->x,(ppit_ls-kb)->y,ppit_ls->x,ppit_ls->y);
      ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

      //STEP 2
      f_vec[0] = ppit_ls->x-(ppit_ls+kf)->x;f_vec[1] = ppit_ls->y-(ppit_ls+kf)->y;
      b_vec[0] = ppit_ls->x-(ppit_ls-kb)->x;b_vec[1] = ppit_ls->y-(ppit_ls-kb)->y;

      
      ROS_INFO("Forward vector:[%f,%f], Backward Vector:[%f,%f]",f_vec[0],f_vec[1],b_vec[0],b_vec[1]);

      dot_product = f_vec[0]*b_vec[0] + f_vec[1]*b_vec[1];
      mag_f = sqrt(pow(f_vec[0],2)+pow(f_vec[1],2));
      mag_b = sqrt(pow(b_vec[0],2)+pow(b_vec[1],2));
      mag_product = mag_f*mag_b;

      ROS_INFO("Dot product: <%f>, Mag product: <%f>",dot_product,mag_product);
      theta_i = constrainAngle(acos(dot_product/mag_product));
      
      if(isinf(theta_i) || isnan(theta_i))
       theta_i = 0;
      ppit_ls->theta = theta_i;
      if (theta_i!=0)
      {
        ROS_INFO("Theta not zero at <%d>:<%f>",(int)std::distance(p_param->begin(),ppit_ls),ppit_ls->theta);
      }

      ROS_INFO("Point parameters for the scan:  <begin:%d,current:%d>:",(int)std::distance(p_param->begin(),begin),(int)std::distance(begin,ppit_ls));
      ROS_INFO("Point <%f,%f,%f>\nPoint type <%d>\nkf <%d>, kb <%d>\ntheta <%f>",ppit_ls->x,ppit_ls->y,ppit_ls->z,ppit_ls->pointType,ppit_ls->kf,ppit_ls->kb,ppit_ls->theta);


      //STEP 3
      if((theta_i < theta_min) && ((kf+kb)>=10))
      {
        ppit_ls->pointType = ppit_ls->pointType*10 + 1;
	ROS_INFO("LaserScanListener::detectLineSegments: Line segment detected <%d>",ppit->pointType);
        
	addTopoFeat(LINE,(ppit_ls->kf+ppit_ls->kb)/2,ppit_ls-kb,ppit_ls+kf);

      }
    }
    ROS_INFO("LaserScanListener::detectLineSegments: Finished!");
    printTopoFeatures();
    return;
  }

  void LaserScanListener::addTopoFeat(int feat, int position, std::vector<PointParamters>::iterator beg, std::vector<PointParamters>::iterator end)
  {
        int size_struct = sizeof(struct topo);
        if(topo_vec.empty())
        {
          topo_vec.resize(1);
          topo_vec[0].feat = feat;
          topo_vec[0].position = position;
          topo_vec[0].beg = beg;
          topo_vec[0].end = end;
        }
        else
        {
          int topo_found = 0;
          int ti;
          for(ti=0;ti<topo_vec.size();ti++)
          {
            if(topo_vec[ti].feat == LINE && (topo_vec[ti].position == position))
            {
              ROS_INFO("This topological feature already present: feature <%d> position <%d> Beginning <%f,%f> End <%f,%f>. Feature number <%d>",topo_vec[ti].feat,topo_vec[ti].position,topo_vec[ti].beg->x,topo_vec[ti].beg->y,topo_vec[ti].end->x,topo_vec[ti].end->y,ti);
              topo_found = 1;
              break;
            }

          }

          if(!topo_found)
          {
              topo_vec.resize(ti);
              temp_topo.feat = feat;
              temp_topo.position = position;
              temp_topo.beg = beg;
              temp_topo.end = end;
              topo_vec.push_back(temp_topo);
              ROS_INFO("New topological feature found: feature <%d> position <%d> Beginning <%f,%f> End <%f,%f>. Feature number <%d>", temp_topo.feat, temp_topo.position, temp_topo.beg->x, temp_topo.beg->y,temp_topo.end->x,temp_topo.end->y,(int)topo_vec.size()/size_struct);
          }
	}
  }

  void LaserScanListener::printTopoFeatures()
  {
    ROS_INFO("------------------------printTopoFeatures---------------------------");
    for (int i = 0;i<topo_vec.size();i++)
    {
      ROS_INFO("Feature <%d>:\nType <%d>\nPosition <%d>\nBeginning <%f,%f> End <%f,%f>",i,topo_vec[i].feat,topo_vec[i].position,topo_vec[i].beg->x,topo_vec[i].beg->y,topo_vec[i].end->x,topo_vec[i].end->y);
      ROS_INFO("Vector associated :<%f,%f>",(topo_vec[i].beg->x-topo_vec[i].end->x),(topo_vec[i].beg->y-topo_vec[i].end->y));
    }
    ROS_INFO("--------------------------------------------------------------------");
  }

  float LaserScanListener::constrainAngle(float x)
  {
    float pi = 3.141579;
    ROS_INFO("Wrapping angle <%f>",x);
    x = fmod(x+pi,2*pi);
    if(x<0)
      x+=(2*pi);
    ROS_INFO("Wrapped angle is <%f>",(x-pi));
    return (x-pi);
  }

  void LaserScanListener::detectCurveAndCorner(std::vector<PointParamters>::iterator begin,std::vector<PointParamters>::iterator end)
  {
    ROS_INFO("LaserScanListener::detectCurveAndCorner: Method entered!");
    int ie, ib;
    float Uc, sum_theta, max_theta, ci;
    int type;
    ie = ib = 2;Uc = 0.5;
    std::vector<PointParamters>::iterator temp_ppit;
    ppit = begin+2;
    for(;ppit<=end-2;ppit++)
    {
      //ROS_INFO("Point cloud <%d> handling now.",(int)std::distance(p_param->begin(),ppit));
      //STEP 4 & 5
      temp_ppit = ppit-ib;

      sum_theta = 0;
      max_theta = temp_ppit->theta;
      ROS_INFO("+++++++++++Printing thestas here!+++++++++++++");
      for(;temp_ppit<=ppit+ie;temp_ppit++)
      {
	if(temp_ppit>p_param->end() || temp_ppit<p_param->begin())
	{
	  ROS_INFO("No more point clouds to handle!");
	  break;
	}
        sum_theta += temp_ppit->theta;
	ROS_INFO("Sum theta <%f>, Current theta <%f>",sum_theta,temp_ppit->theta);
	if(max_theta<temp_ppit->theta)
	  max_theta = temp_ppit->theta;
      }
      ROS_INFO("Max theta <%f>",max_theta);
      ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++");

      if(!max_theta)
        ci = (sum_theta/5)/max_theta;
      else
        ci = 0.0;
      if (ci>Uc)
      {
        ppit->pointType = CURVE;
	ppit->ci = ci;
	//rho = 1/((sum_theta/5)*c);
	//xc = (pcl_iter-ib)->x + rho*cos(sum_theta);
	//yc = (pcl_iter-ib)->y + rho*cos(sum_theta);
	addTopoFeat(CURVE,std::distance(p_param->begin(),ppit),ppit-ib,ppit+ie);
      }
      else
      {
        ROS_INFO("CI <%f> not greater!",ci);
      }

    }
    ROS_INFO("----------------------AFTER DET CURVES-----------------------------");
    printTopoFeatures();
    ROS_INFO("-------------------------------------------------------------------");

    //STEP 6
    /*for(pcl_iter=pcl_cloud->begin(),ppit=p_param->begin();pcl_iter<pcl_cloud->end();ppit++,pcl_iter++)
    {
      //if(pcl_iter == NULL)
      //{
	//ROS_INFO("LaserScanListener::detectCurveAndCorner: Null point cloud!");
	//return;
      //}
      //if(ppit == NULL)
      //{
        //ROS_INFO("LaserScanListener::detectCurveAndCorner: Null param detected!");
        //return;
      //}
      if((ppit->ci>(ppit+1)->ci) && (ppit->ci>(ppit-1)->ci))
      {
        if(ppit->theta>theta_min)
	{
	  ROS_INFO("LaserScanListener::detectCurveAndCorner: Corner detected!");
	  publishPoint(pcl_iter,CORNER);
	}
      }
    }*/
    ROS_INFO("LaserScanListener::detectCurveAndCorner: Finished!");
  }

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

    p.points.push_back(gp);
    ROS_INFO("Publishing the point of type <%d>:<%f,%f,%f>",point_type,gp.x,gp.y,gp.z);
    p_pub.publish(p);
  }

  void LaserScanListener::publishPoint(std::vector<PointParamters>::iterator iter,int point_type)
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

    p.points.push_back(gp);
    ROS_INFO("Publishing the point of type <%d>:<%f,%f,%f>",point_type,gp.x,gp.y,gp.z);
    p_pub.publish(p);
  }

  void LaserScanListener::detectIntersection()
  {
    float per_angle, par_angle, acute_angle, obt_angle, right_turn_angle, left_turn_angle;
    per_angle = 1.57;
    par_angle = 3.14;
    acute_angle = 0.516;
    obt_angle = 2.62;
    right_turn_angle = 2.0;
    left_turn_angle = 1.0;
    float sigma = 0.35;
    bool t_int,four_int,per_flag,par_flag,obt_flag,acute_flag;
    t_int = four_int = obt_flag = acute_flag = per_flag = par_flag = 0;
    ROS_INFO("LaserScanListener::detectIntersection: Method entered!");
    if(topo_vec.empty())
    {
      ROS_INFO("NOthing to detect!");
      return;
    }

    ROS_INFO("BEGIN HARSHA");
    struct topo temp_topo;
    for(int i=0;i<topo_vec.size();i++)
    {
      if((topo_vec[i].beg->y < 0) && (topo_vec[i].end->y < 0))
      {
        ROS_INFO("Found the line to the right of robot <%f,%f>-<%f,%f>",topo_vec[i].beg->x,topo_vec[i].beg->y,topo_vec[i].end->x,topo_vec[i].end->y);
	if (robot.left.empty())
	{
	  robot.left.resize(1);
	  robot.left[robot.left.size()-1].feat = topo_vec[i].feat;
	  robot.left[robot.left.size()-1].position = topo_vec[i].position;
	  robot.left[robot.left.size()-1].beg = topo_vec[i].beg;
	  robot.left[robot.left.size()-1].end = topo_vec[i].end;
	  //robot.left.push_back(temp_topo);
	  ROS_INFO("Assigned new to left!");
	}
	else
	{
          robot.left.resize(robot.left.size()+1);
          robot.left[robot.left.size()-1].feat = topo_vec[i].feat;
          robot.left[robot.left.size()-1].position = topo_vec[i].position;
          robot.left[robot.left.size()-1].beg = topo_vec[i].beg;
          robot.left[robot.left.size()-1].end = topo_vec[i].end;
          //robot.left.push_back(temp_topo);
          ROS_INFO("Assigned new to left!");
	}
      }
      else if((topo_vec[i].beg->y > 0) && (topo_vec[i].end->y > 0))
      {
        ROS_INFO("Found the line to the left of robot <%f,%f>-<%f,%f>",topo_vec[i].beg->x,topo_vec[i].beg->y,topo_vec[i].end->x,topo_vec[i].end->y);
	if (robot.right.empty())
	{
	  robot.right.resize(1);
          robot.right[robot.right.size()-1].feat = topo_vec[i].feat;
          robot.right[robot.right.size()-1].position = topo_vec[i].position;
          robot.right[robot.right.size()-1].beg = topo_vec[i].beg;
          robot.right[robot.right.size()-1].end = topo_vec[i].end;
	  //robot.right.push_back(temp_topo);
	  ROS_INFO("Assigned to right!");
	}
        else
        {
          robot.right.resize(robot.right.size()+1);
          robot.right[robot.right.size()-1].feat = topo_vec[i].feat;
          robot.right[robot.right.size()-1].position = topo_vec[i].position;
          robot.right[robot.right.size()-1].beg = topo_vec[i].beg;
          robot.right[robot.right.size()-1].end = topo_vec[i].end;
          //robot.right.push_back(temp_topo);
          ROS_INFO("Assigned new to right!");
        }
      }
      else
      {
        ROS_INFO("Found the line to the center of robot <%f,%f>-<%f,%f>",topo_vec[i].beg->x,topo_vec[i].beg->y,topo_vec[i].end->x,topo_vec[i].end->y);
	if (robot.center.empty())
	{
	  robot.center.resize(1);
          robot.center[robot.center.size()-1].feat = topo_vec[i].feat;
          robot.center[robot.center.size()-1].position = topo_vec[i].position;
          robot.center[robot.center.size()-1].beg = topo_vec[i].beg;
          robot.center[robot.center.size()-1].end = topo_vec[i].end;
	  //robot.center.push_back(temp_topo);
	  ROS_INFO("Assigned to the center <%f,%f>-<%f,%f>!",robot.center[robot.center.size()-1].beg->x,robot.center[robot.center.size()-1].beg->y,robot.center[robot.center.size()-1].end->x,robot.center[robot.center.size()-1].end->y);
	}
        else
        {
          robot.center.resize(robot.center.size()+1);
          robot.center[robot.center.size()-1].feat = topo_vec[i].feat;
          robot.center[robot.center.size()-1].position = topo_vec[i].position;
          robot.center[robot.center.size()-1].beg = topo_vec[i].beg;
          robot.center[robot.center.size()-1].end = topo_vec[i].end;
          robot.center.push_back(temp_topo);
          //ROS_INFO("Assigned new to center <%f,%f>-<%f,%f>!",robot.center[0].beg->x,robot.center[0].beg->y,robot.center[0].end->x,robot.center[0].end->y);
        }
      }
    }

    //Starting from right!
    if(robot.right.size()>1)
    {
      //Establishing relationship between the nearest right and the farthest right
      int least_i = 0;

      float min_y; 
      if(robot.right[0].beg->y>robot.right[0].end->y)
        min_y = robot.right[0].end->y;
      else	
	min_y = robot.right[0].beg->y;
      for(int i=1;i<robot.right.size();i++)
      {
        if(robot.right[i].beg->y>min_y || robot.right[i].end->y>min_y)
	{
	  least_i = i;
	  if(robot.right[i].beg->y>robot.right[i].end->y)
	    min_y = robot.right[i].end->y;
	  else
	    min_y = robot.right[i].beg->y;
	}
      }
      float vec1[2],vec2[2],angle;
      float mag_prod,dot_prod;
      vec1[0] = robot.right[least_i].beg->x-robot.right[least_i].end->x;
      vec1[1] = robot.right[least_i].beg->y-robot.right[least_i].end->y;


      for(int i=0;i<robot.right.size();i++)
      {
        if(i == least_i)
	  continue;
	vec2[0] = robot.right[i].beg->x-robot.right[i].end->x;
	vec2[1] = robot.right[i].beg->y-robot.right[i].end->y;
	dot_prod = vec1[0]*vec2[0] + vec1[1]*vec2[1];
	mag_prod = sqrt(pow(vec1[0],2)+pow(vec1[1],2))*sqrt(pow(vec2[0],2)+pow(vec2[1],2));
	angle = acos(dot_prod/mag_prod);
	ROS_INFO("Angle made is:<%f> between feature <%d> and <%d>",angle,robot.right[least_i].position,robot.right[i].position);

      }

    }

    if(robot.left.size()>1)
    {
      //Establishing relationship between the nearest right and the farthest right
      int least_i = 0;
      float min_y = (robot.left[0].beg->y<robot.left[0].end->y)?robot.left[0].end->y:robot.left[0].beg->y;
      for(int i=1;i<robot.left.size();i++)
      {
        if(robot.left[i].beg->y<min_y || robot.left[i].end->y<min_y)
        {
          least_i = i;
          min_y = (robot.left[i].beg->y<robot.left[i].end->y)?robot.left[i].end->y:robot.left[i].beg->y;
        }
      }
      float vec1[2],vec2[2],angle; 
      float mag_prod,dot_prod;
      vec1[0] = robot.left[least_i].beg->x-robot.left[least_i].end->x;
      vec1[1] = robot.left[least_i].beg->y-robot.left[least_i].end->y;

      for(int i=0;i<robot.left.size();i++)
      {
        if(i == least_i)
          continue;
        vec2[0] = robot.left[i].beg->x-robot.left[i].end->x;
        vec2[1] = robot.left[i].beg->y-robot.left[i].end->y;
        dot_prod = vec1[0]*vec2[0] + vec1[1]*vec2[1];
        mag_prod = sqrt(pow(vec1[0],2)+pow(vec1[1],2))*sqrt(pow(vec2[0],2)+pow(vec2[1],2));
        angle = acos(dot_prod/mag_prod);
        ROS_INFO("Angle made is:<%f> between feature <%d> and <%d>",angle,robot.left[least_i].position,robot.left[i].position);
      }

    }

    if(!robot.center.empty())
    {
      float vec1[2],vec2[2],vec3[2],angle;
      float mag_prod,dot_prod;
      vec1[0] = robot.center[0].beg->x-robot.center[0].end->x;
      vec1[1] = robot.center[0].beg->y-robot.center[0].end->y;
      if(robot.right[robot.right.size()-1].end->y>robot.center[0].beg->y)
      {
        ROS_INFO("Space of <%f> between center line and last right line",robot.right[robot.right.size()-1].end->y-robot.center[0].beg->y);
      }
      vec2[0] = robot.right[robot.right.size()-1].beg->x-robot.right[robot.right.size()-1].end->x;
      vec2[1] = robot.right[robot.right.size()-1].beg->y-robot.right[robot.right.size()-1].end->y;
      dot_prod = vec1[0]*vec2[0] + vec1[1]*vec2[1];
      mag_prod = sqrt(pow(vec1[0],2)+pow(vec1[1],2))*sqrt(pow(vec2[0],2)+pow(vec2[1],2));
      angle = acos(dot_prod/mag_prod);
      ROS_INFO("Angle made between right and center: <%f>",angle);

      if(robot.left[robot.left.size()-1].end->y>robot.center[0].end->y)
      {
        ROS_INFO("Space of <%f> between center line and last left line",robot.left[robot.left.size()-1].end->y-robot.center[0].end->y);
      }
      vec3[0] = robot.left[robot.left.size()-1].beg->x-robot.left[robot.left.size()-1].end->x;
      vec3[1] = robot.left[robot.left.size()-1].beg->y-robot.left[robot.left.size()-1].end->y;
      dot_prod = vec1[0]*vec3[0] + vec1[1]*vec3[1];
      mag_prod = sqrt(pow(vec1[0],2)+pow(vec1[1],2))*sqrt(pow(vec3[0],2)+pow(vec3[1],2));
      angle = acos(dot_prod/mag_prod);
      ROS_INFO("Angle made between left and center: <%f>",angle);
    }

    ROS_INFO("END HARSHA");

    for(int i=0;i<topo_vec.size();i++)
    {
      float vec1[2],vec2[2],angle;
      float mag_prod,dot_prod;
      vec1[0] = topo_vec[i].beg->x-topo_vec[i].end->x;
      vec1[1] = topo_vec[i].beg->y-topo_vec[i].end->y;
      for(int j=0;j<topo_vec.size();j++)
      {
        if(i==j)
	  continue;
	vec2[0] = topo_vec[j].beg->x-topo_vec[j].end->x;
	vec2[1] = topo_vec[j].beg->y-topo_vec[j].end->y;
	dot_prod = vec1[0]*vec2[0] + vec1[1]*vec2[1];
	mag_prod = sqrt(pow(vec1[0],2)+pow(vec1[1],2))*sqrt(pow(vec2[0],2)+pow(vec2[1],2));
	angle = acos(dot_prod/mag_prod);
	if(!isnan(angle)){
        if((angle<(acute_angle+sigma) && angle>(acute_angle-sigma)))
	{
	  four_int = 1;
	  acute_flag = 1;
        }
        if((angle<(obt_angle+sigma) && angle>(obt_angle-sigma)))
        {
          four_int = 1;
	  obt_flag = 1;
        }
	if(angle<(per_angle+sigma) && angle>(per_angle-sigma))
	{
	  t_int = 1;
	  per_flag = 1;
	}
	if(angle<(par_angle+sigma) && angle>par_angle-sigma)
	{
	  //t_int = 1;
	  par_flag = 1;
	}
        if(angle<(right_turn_angle+sigma) && angle>right_turn_angle-sigma)
          ROS_INFO("INTERSECTION : RIGHT TURN");
	if(angle<(left_turn_angle+sigma) && angle>left_turn_angle-sigma)
	  ROS_INFO("INTERSECTION : LEFT TURN");
	ROS_INFO("Angle between <%f,%f>-<%f,%f> and <%f,%f>-<%f,%f> = %f",topo_vec[i].beg->x,topo_vec[i].beg->y,topo_vec[i].end->x,topo_vec[i].end->y,topo_vec[j].beg->x,topo_vec[j].beg->y,topo_vec[j].end->x,topo_vec[j].end->y,angle);
      }
      }
    }

      if (per_flag && par_flag && !four_int)
        ROS_INFO("INTERSECTION : T-INTERSECTION");
      if (acute_flag && obt_flag && !t_int)
        ROS_INFO("INTERSECTION : FOURWAY INTERSETION");


  }

  int LaserScanListener::determineRelation(float angle,std::vector<PointParameters>::iterator beg1,std::vector<PointParamters>::iterator end1,std::vector<PointParamters>::iterator beg2,std::vector<PointParamters>::iterator end2)
  {
        if(!isnan(angle)){
        if((angle<(acute_angle+sigma) && angle>(acute_angle-sigma)))
        {
          return(acute);
        }
        if((angle<(obt_angle+sigma) && angle>(obt_angle-sigma)))
        {
          return(obtuse);
        }
        if(angle<(per_angle+sigma) && angle>(per_angle-sigma))
        {
          return(per);
        }
        if(angle<(par_angle+sigma) && angle>par_angle-sigma)
        {
          return(par);
        }
        return UNKNOWN;

  }

  float LaserScanListener::computeEuclidDist(std::vector<PointParamters>::iterator pcl1,std::vector<PointParamters>::iterator pcl2)
  {
    float dist;
    float dist_x = pcl1->x - pcl2->x;
    float dist_y = pcl1->y - pcl2->y;

    dist = pow(dist_x,2) + pow(dist_y,2);
    dist = sqrt(dist);
    //ROS_INFO("Computing euclidean distance between <%f,%f> and <%f,%f>:<%f>",pcl1->x,pcl1->y,pcl2->x,pcl2->y,dist);
    return dist;
  }

  float LaserScanListener::computeEuclidDist(pcl::PointCloud<pcl::PointXYZ>::iterator pcl1,pcl::PointCloud<pcl::PointXYZ>::iterator pcl2)
  {
    float dist;
    float dist_x = pcl1->x - pcl2->x;
    float dist_y = pcl1->y - pcl2->y;
    ROS_INFO("Computing euclidean distance between <%f,%f> and <%f,%f>",pcl1->x,pcl1->y,pcl2->x,pcl2->y);

    dist = pow(dist_x,2) + pow(dist_y,2);
    dist = sqrt(dist);
    return dist;
  }

  LaserScanListener::~LaserScanListener(){

  }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "scannode");
ros::NodeHandle n("laser_scan");
LaserScanListener lsListener(n);
ros::spin();
return 0;
}
