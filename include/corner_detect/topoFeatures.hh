#include "std_msgs/String.h"
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>

#define PER_ANG 1.57
#define PAR_ANG 3.14
#define ACU_ANG 0.516
#define OBT_ANG 2.62
#define RIG_TUR_ANG 2.0
#define LEF_TUR_ANG 3.0
#define SIGMA 0.4
#define ANG_SIGMA 0.1
#define LIN_SIGMA 0.50
#define GAP_SIGMA 1.5

namespace inter_det
{
   struct topo
    {
      int feat;
      int position;
      float beg_x;
      float beg_y;
      float end_x;
      float end_y;
      float cur_time;
    };
   struct node
    {
            int type;
            struct topo info;
            struct relation *fr;
	    struct relation *br;
    };

    struct relation
    {
            float angle;
            float space_x, space_y;
            bool side_gap, same_gap;
            int rel_to;
	    struct node *next;
    };

    struct ret_code
    {
	    bool gap;
	    int rel;
    };

  class TopoFeature
  {
    enum REL_ENT { RIGHT, LEFT, FRONT, BACK, UNKNOWN, T, FOUR };
    enum REL_POI { BEG=1, END, UNK, NONE, PAR };

    struct intersection
    {
	    int type;
	    tf::Pose p;
    };

    struct node *node_ref_, *node_head_;

    std::vector<topo> topo_vec_;
    std::vector<intersection> pose_;

    enum GAP_T{FORW_G,LEFT_G,RIGHT_G};
    enum SIDE{QUAD,MULT};
    enum features{BREAKPOINT,LINE,CURVE,CORNER};
    enum intersec{TI,RI,RT,LI,LT};

    tf::Transform cur_tf_;

	  public:
    void setCurrentTf(tf::Transform t);
    void addTopoFeat(int feat, int pos, float beg_x, float beg_y, float end_x,float  end_y,float time);
    void printTopoFeatures();
    int identifyIntersection();
    void updateFlags(float angle,struct topo seg1,struct topo seg2,int comp_flag,int side_flag);
    void initFlags();
    void clearTopoFeatures(int);
    int buildRelation();
    int checkLineIntersection(struct topo, struct topo);
    int logIntersection( bool, bool, bool );
    void printRelation();
    int checkParlell(struct topo, struct topo);
    float dist(float x1, float y1, float x2, float y2)
    {
	    return(sqrt(pow((x2 - x1),2) + pow((y2 - y1),2)));
    }
  };

}
