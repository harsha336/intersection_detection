#include <stdio.h>

KF::KF(float pos_x,float pos_y)
{
  ROS_INFO("Initialized the Kallmann filter!");
  x = 0;y = 0;
  inx = pos_x;iny = pos_y;
  poi_flag = 0;
}

void KF::updatePositionAndPOI(float r,float theta)
{
  x = x+r*cos(theta);
  y = y+r*sin(theta);
  inx = inx+r*cos(theta);
  iny = iny+r*sin(theta);
  ROS_INFO("The new position of the robot is <%f,%f> and marker is <%f,%f>",x,y,inx,iny);
}

void KF::checkPOI(float x,float y)
{
  if(!poi_flag)
  {
    ROS_INFO("SOmehting new <%f,%f>",x,y);
    inx = x;
    iny = y;
    return;
  }

  if((inx+sigma>x) && (inx-sigma<x) && (iny+sigma>y) && (iny-sigma<y))
  {
    ROS_INFO("Same point <%f,%f> as <%f,%f>",inx,iny,x,y);
  }
  else
  {
    ROS_INFO("This is new <%f,%f> at <%f,%f>",x,y,pos_x,pos_y);
    inx = x;iny = y;
    poi_flag = 1;
  }
  
}

KF::~KF()
{
  ROS_INFO("Finished handleing the POI <%f,%f>",inx,iny);
}
