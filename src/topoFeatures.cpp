#include <corner_detect/topoFeatures.hh>

void inter_det::TopoFeature::addTopoFeat(int feat, int pos, float beg_x, float beg_y, float end_x,float  end_y,float time)
{
  ROS_INFO("TopoFeature::addTopoFeat: Adding topo feature: <%d,%d,(%f,%f),(%f,%f),%f>",feat,pos,beg_x,beg_y,end_x,end_y,time);
  int size_struct = sizeof(struct topo);
  struct topo temp_topo;
  bool topo_found = false;
  unsigned int ti;
  if(topo_vec_.empty())
    ti = 1;
  else
  {
    for(ti = 0;ti < topo_vec_.size();ti++)
    {
      if(topo_vec_[ti].feat == LINE && (topo_vec_[ti].position == pos))
      {
        int prev_type = topo_vec_[ti].feat;
	if(topo_vec_[ti].feat == LINE && feat == CURVE)
	  topo_vec_[ti].feat == feat;
        ROS_INFO("Topo feat already present: feature <%d> position <%d> Beginning <%f,%f> End <%f,%f>, Feature number <%d>",topo_vec_[ti].feat,topo_vec_[ti].position,topo_vec_[ti].beg_x,topo_vec_[ti].beg_y,topo_vec_[ti].end_x,topo_vec_[ti].end_y,ti);
	topo_found = true;
	break;
      }
    }
  }

  if(!topo_found)
  {
      ROS_INFO("Topo adding!");
      //topo_vec_.resize(ti);
      temp_topo.feat = feat;
      temp_topo.position = pos;
      temp_topo.beg_x = beg_x;
      temp_topo.beg_y = beg_y;
      temp_topo.end_x = end_x;
      temp_topo.end_y = end_y;
      temp_topo.cur_time = time;
      topo_vec_.push_back(temp_topo);
      ROS_INFO("New topo feat added: Feature <%d> position <%d> Beginning <%f,%f> End <%f,%f>, Feature number <%d>",temp_topo.feat,temp_topo.position,temp_topo.beg_x,temp_topo.beg_y,temp_topo.end_x,temp_topo.end_y,(int)topo_vec_.size()/size_struct);
  }
}

void inter_det::TopoFeature::setCurrentTf(tf::Transform t)
{
	ROS_INFO("TopoFeature::setCurrentTf: Setting the current tf to <%f,%f,%f,%f>",t.getOrigin().x(),t.getOrigin().y(),t.getOrigin().z(),tf::getYaw(t.getRotation()));
	cur_tf_ = t;
}

void inter_det::TopoFeature::clearTopoFeatures()
{
	ROS_INFO("TopoFeature::clearTopoFeatures: Clearing all the topological features!");
	topo_vec_.clear();
	node_ref_ = node_head_;
	int end;
	while( true )
	{
		end = node_ref_->type;
		ROS_INFO_STREAM( "TopoFeature::clearTopoFeatures: Deleting node: " <<
				node_ref_->type);
		node_head_ = node_ref_->fr->next;
		delete node_ref_;
		if(end == END) 
			break;
		else
			node_ref_ = node_head_;
	}
	return;
}

void inter_det::TopoFeature::printTopoFeatures()
{
  ROS_INFO("------------------------printTopoFeatures----------------------------");
  for(unsigned int i = 0;i < topo_vec_.size();i++)
  {
    ROS_INFO("Feature <%d>:\nType <%d>\nPosition <%d>\nBeginning <%f,%f> End <%f,%f>",i,topo_vec_[i].feat,topo_vec_[i].position,topo_vec_[i].beg_x,topo_vec_[i].beg_y,topo_vec_[i].end_x,topo_vec_[i].end_y);
  }
  ROS_INFO("---------------------------------------------------------------------");
}

int inter_det::TopoFeature::checkLineIntersection(struct topo l1, struct topo l2)
{
	float a1 = l1.end_y - l1.beg_y;
	float b1 = l1.beg_x - l1.end_x;
	float c1 = a1*l1.beg_x + b1*l1.beg_y;

	float a2 = l2.end_y - l2.beg_y;
	float b2 = l2.beg_x - l2.end_x;
	float c2 = a2*l2.beg_x + b2*l2.beg_y;

	float det = a1*b2 - a2*b1;
	float x, y;

	if (det == 0)
	{
		ROS_INFO_STREAM ("TopoFeature::checkLineIntersection: Should not come here as not parlell");
	}
	else
	{
		x = (b2*c1 - b1*c2)/det;
		y = (a1*c2 - a2*c1)/det;
	}

	if ( ((std::min(l1.beg_x - LIN_SIGMA, l1.end_x + LIN_SIGMA) <= x && x <= std::max(l1.beg_x - LIN_SIGMA, l1.end_x + LIN_SIGMA)) &&
	      (std::min(l1.beg_y - LIN_SIGMA, l1.end_y + LIN_SIGMA) <= y && y <= std::max(l1.beg_y - LIN_SIGMA, l1.end_y + LIN_SIGMA))) ||
	     ((std::min(l2.beg_x - LIN_SIGMA, l2.end_x + LIN_SIGMA) <= x && x <= std::max(l2.beg_x - LIN_SIGMA, l2.end_x + LIN_SIGMA)) &&
	      (std::min(l2.beg_y - LIN_SIGMA, l2.end_y + LIN_SIGMA) <= y && y <= std::max(l2.beg_y - LIN_SIGMA, l2.end_y + LIN_SIGMA))))
		return FRONT;	
	else if( x > l1.end_x || y > l1.end_y )
		if( x > l2.beg_x || y > l2.beg_y )
			return LEFT;
		else if( x < l2.beg_x || y < l2.beg_y )
			return RIGHT;
	else if( x < l1.end_x || y < l1.end_y )
		return LEFT;
}

int inter_det::TopoFeature::checkParlell(struct topo l1, struct topo l2)
{
	float para_x, para_y, begx, begy;
	bool rev;
	int sign;
	int t = 0.01;
	para_x = l1.end_x - l1.beg_x;
	para_y = l1.end_y - l1.beg_y;
	begx = l1.beg_x; begy = l1.beg_y;
	(begx < l2.beg_x)? sign = -1 : sign = 1;
	while( sign*begx < l2.beg_x )
	{
		ROS_INFO_STREAM( "TopoFeature::checkParlell: In while loop!");
		begx = begx + para_x*t;
		begy = begy + para_y*t;
		if((begx - l2.beg_x) < LIN_SIGMA && (begy - l2.beg_y) < LIN_SIGMA)
		{
			ROS_INFO_STREAM( "TopoFeature::checkParlell: They meet hence front" );
			return FRONT;
		}
	}
	return LEFT;
}
int inter_det::TopoFeature::buildRelation()
{
	ROS_INFO_STREAM( "inter_det::TopoFeature::buildRelation: Method entered");
	int cur, next;
	struct relation *r;
	struct node *new_node, *next_new_node;
	if(topo_vec_.empty())
	{
		ROS_INFO_STREAM( "TopoFeature::buildRelation: No features to detect returning" );
		return -999;
	}

	float vec1[2], vec2[2];
	float dot_prod, mag_prod;
	int temp; std::string temp_str;
	for (unsigned int i = 0;i < topo_vec_.size();i++)
	{
		cur = i;
		struct relation* fr = (struct relation*)malloc(sizeof(struct relation));
		struct relation* br = (struct relation*)malloc(sizeof(struct relation));

		if ( i == (topo_vec_.size() - 1))
		{
			next = 0;
			new_node = node_ref_;
			next_new_node = node_head_;
			new_node->type = END;
			ROS_INFO_STREAM( "TopoFeature::buildRelation: New node type: " << new_node->type
					  << "\nnew next node type: " << next_new_node->type << "last" );
		}
		else if(i == 0)
		{
			next = i + 1;
			new_node = (struct node*)malloc(sizeof(struct node));
			next_new_node = (struct node*)malloc(sizeof(struct node));
			new_node->type = BEG;
			next_new_node->type = NONE;
			new_node->info = topo_vec_[cur];
			next_new_node->info = topo_vec_[next];
			node_ref_ = new_node;
			node_head_ = new_node;
			ROS_INFO_STREAM( "TopoFeature::buildRelation: New node type: " << new_node->type
                                          << "\nnew next node type: " << next_new_node->type << "first" );
		}
		else
		{
			next = i + 1;
			next_new_node = (struct node*)malloc(sizeof(struct node));
			new_node = node_ref_;
			next_new_node->info = topo_vec_[next];
			next_new_node->type = NONE;
			ROS_INFO_STREAM( "TopoFeature::buildRelation: New node type: " << new_node->type
                                          << "\nnew next node type: " << next_new_node->type << "mid" );
		}

		fr->next = next_new_node;
		br->next = new_node;

		br->same_gap = br->side_gap = fr->same_gap = fr->side_gap = false;

		br->space_x = fr->space_x = topo_vec_[next].beg_x - topo_vec_[cur].end_x;
		br->space_y = fr->space_y = topo_vec_[next].beg_y - topo_vec_[cur].end_y;
		
		float mag;

		vec1[0] = topo_vec_[cur].beg_x - topo_vec_[cur].end_x;
		vec1[1] = topo_vec_[cur].beg_y - topo_vec_[cur].end_y;
		mag = sqrt(pow(vec1[0],2) + pow(vec1[1],2));
		ROS_INFO_STREAM( "TopoFeature::buildRelation: Vec 1: " << vec1[0] << "," << vec1[1] );
		ROS_INFO_STREAM( "TopoFeature::buildRelation: Mag: " << mag );
		vec1[0] = vec1[0]/mag; vec1[1] = vec1[1]/mag;

		vec2[0] = topo_vec_[next].beg_x - topo_vec_[next].end_x;
		vec2[1] = topo_vec_[next].beg_y - topo_vec_[next].end_y;
		mag = sqrt(pow(vec2[0],2) + pow(vec2[1],2));
                ROS_INFO_STREAM( "TopoFeature::buildRelation: Vec 2: " << vec2[0] << "," << vec2[1] );
		ROS_INFO_STREAM( "TopoFeature::buildRelation: Mag: " << mag );
		vec2[0] = vec2[0]/mag; vec2[1] = vec2[1]/mag;

		dot_prod = vec1[0]*vec2[0] + vec1[1]*vec2[1];
		mag_prod = sqrt(pow(vec1[0],2) + pow(vec1[1],2) *
				pow(vec2[0],2) + pow(vec2[1],2));
		ROS_INFO_STREAM( "TopoFeature::buildRelation: First line: <" << topo_vec_[cur].beg_x <<
								         "," << topo_vec_[cur].beg_y <<
									"><" << topo_vec_[cur].end_x << 
									 "," << topo_vec_[cur].end_y << 
									 ">");
		ROS_INFO_STREAM(" TopoFeature::buildRelation: Sec Line: <" << topo_vec_[next].beg_x <<
								       "," << topo_vec_[next].beg_y <<
								      "><" << topo_vec_[next].end_x <<
								       "," << topo_vec_[next].end_y <<
								       ">" );
		ROS_INFO_STREAM( "TopoFeature::buildRelation: Vec 1: " << vec1[0] << "," << vec1[1] );
		ROS_INFO_STREAM( "TopoFeature::buildRelation: Vec 2: " << vec2[0] << "," << vec2[1] );
		ROS_INFO_STREAM( "TopoFeature::buildRelation: Dot prod: " << dot_prod <<
				                             "Mag prod: " << mag_prod );

		if(dot_prod/mag_prod < -1)
			br->angle = fr->angle = M_PI;
		else if(dot_prod/mag_prod > 1)
			br->angle = fr->angle = 0;
		else
			fr->angle = br->angle = acos(dot_prod/mag_prod);

		if(std::abs(fr->angle - PER_ANG) < ANG_SIGMA)
		{
			ROS_INFO_STREAM( "TopoFeature::buildRelation: Found a perpendicular angle!" );
			if( checkLineIntersection(topo_vec_[cur], topo_vec_[next]) == FRONT )
			{
				fr->rel_to = FRONT;
				br->rel_to = BACK;
				new_node->fr = fr;
				next_new_node->br = br;
				temp = FRONT;
				temp_str = "FRONT";
				if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
                                                topo_vec_[next].beg_x, topo_vec_[next].beg_y) > GAP_SIGMA)
                                        br->same_gap = fr->same_gap = true;

			}
			else if( checkLineIntersection(topo_vec_[cur], topo_vec_[next]) == LEFT )
			{
				fr->rel_to = LEFT;
				br->rel_to = RIGHT;
				new_node->fr = fr;
				next_new_node->br = br;
				temp = LEFT;
				temp_str = "LEFT";
				if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
                                                topo_vec_[next].beg_x, topo_vec_[next].beg_y) > GAP_SIGMA)
                                        br->side_gap = fr->side_gap = true;
			}
			else if(checkLineIntersection(topo_vec_[cur], topo_vec_[next]) == RIGHT)
			{
				fr->rel_to = RIGHT;
				br->rel_to = LEFT;
				new_node->fr = fr;
				next_new_node->br = br;
				temp = RIGHT;
				temp_str = "RIGHT";
				if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
                                                topo_vec_[next].beg_x, topo_vec_[next].beg_y) > GAP_SIGMA)
                                        br->side_gap = fr->side_gap = true;
			}
		}
		else if( std::abs(fr->angle - PAR_ANG) < ANG_SIGMA || (std::abs(fr->angle) < ANG_SIGMA))
		{
			ROS_INFO_STREAM( "TopoFeature::buildRelation: Found a parlell angle!" );
			if( checkParlell(topo_vec_[cur], topo_vec_[next]) == FRONT )
			{
				fr->rel_to = FRONT;
				br->rel_to = BACK;
				new_node->fr = fr;
				next_new_node->br = br;
				temp = FRONT;
				temp_str = "FRONT";
				if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
						topo_vec_[next].beg_x, topo_vec_[next].beg_y) > GAP_SIGMA)
					br->same_gap = fr->same_gap = true;

			}
			else if( checkParlell(topo_vec_[cur], topo_vec_[next]) == LEFT )
			{
				fr->rel_to = LEFT;
                                br->rel_to = RIGHT;
                                new_node->fr = fr;
                                next_new_node->br = br;
                                temp = LEFT;
                                temp_str = "LEFT";
                                if(dist(topo_vec_[cur].end_x, topo_vec_[cur].end_y,
                                                topo_vec_[next].beg_x, topo_vec_[next].beg_y) > GAP_SIGMA)
                                        br->side_gap = fr->side_gap = true;
			}
			else
			{
				ROS_INFO_STREAM( "TopoFeature::buildRelation: Parlell angles should not be different than the current" );
				return -999;
			}
		}
		else
		{
			ROS_INFO_STREAM( "TopoFeature::buildRelation: Found a wierd angle: " << r->angle);
			return -999;
		}
		ROS_INFO_STREAM( "TopoFeature::buildRelation: Node [" << node_ref_->type <<
                                 "]: Info: <" << node_ref_->info.beg_x << "," << node_ref_->info.beg_y
                                 << "><" << node_ref_->info.end_x << "," << node_ref_->info.end_y
                                 << ">");
		ROS_INFO_STREAM( "TopoFeature::buildRelation: Relation [" <<
                                                  temp_str << "]: angle: " << node_ref_->fr->angle <<
                                                  " space_x: " << node_ref_->fr->space_x <<
                                                  " space_y: " << node_ref_->fr->space_y <<
                                                  " side_gap: " << node_ref_->fr->side_gap <<
                                                  " same_gap: " << node_ref_->fr->same_gap << 
						  " next type: " << node_ref_->fr->next->type );

		node_ref_ = next_new_node;
	}
	return 1;
}

void inter_det::TopoFeature::printRelation()
{
	ROS_INFO_STREAM( "TopoFeature::printRelation: Method entered!" );
	if( node_head_ == NULL )
	{
		ROS_INFO_STREAM( "TopoFeature::printRelation: Nothing to print" );
		return;
	}

	node_ref_ = node_head_;
	int ent;
	std::string rel = "NONE";
	while( node_ref_ != NULL )
	{
		ROS_INFO_STREAM( "TopoFeature::printRelation: Node [" << node_ref_->type <<
				 "]: Info: <" << node_ref_->info.beg_x << "," << node_ref_->info.beg_y
				 << "><" << node_ref_->info.end_x << "," << node_ref_->info.end_y
				 << ">");
		if( node_ref_->fr->rel_to == RIGHT ) rel = "RIGHT";
		else if( node_ref_->fr->rel_to == LEFT ) rel = "LEFT";
		else if( node_ref_->fr->rel_to == FRONT ) rel = "FRONT";
		else if( node_ref_->fr->rel_to == BACK ) rel = "BACK";
		ROS_INFO_STREAM( "TopoFeature::printRelation: Relation [" <<
					rel << "]: angle: " << node_ref_->fr->angle <<
					" space_x: " << node_ref_->fr->space_x << 
					" space_y: " << node_ref_->fr->space_y <<
					" side_gap: " << node_ref_->fr->side_gap <<
					" same_gap: " << node_ref_->fr->same_gap );
		if( node_ref_->fr->next->type == BEG )
			node_ref_ = NULL;
		else node_ref_ = node_ref_->fr->next;

	}

	ROS_INFO_STREAM( "TopoFeature::printRelation: Printing backwards!");
	node_ref_ = node_head_->br->next;
	while( node_ref_ != NULL )
        {
                ROS_INFO_STREAM( "TopoFeature::printRelation: Node [" << node_ref_->type <<
                                 "]: Info: <" << node_ref_->info.beg_x << "," << node_ref_->info.beg_y
                                 << "><" << node_ref_->info.end_x << "," << node_ref_->info.end_y
                                 << ">");
                if( node_ref_->br->rel_to == RIGHT ) rel = "RIGHT";
                else if( node_ref_->br->rel_to == LEFT ) rel = "LEFT";
                else if( node_ref_->br->rel_to == FRONT ) rel = "FRONT";
                else if( node_ref_->br->rel_to == BACK ) rel = "BACK";
                ROS_INFO_STREAM( "TopoFeature::printRelation: Relation [" <<
                                        rel << "]: angle: " << node_ref_->br->angle <<
                                        " space_x: " << node_ref_->br->space_x <<
                                        " space_y: " << node_ref_->br->space_y <<
                                        " side_gap: " << node_ref_->br->side_gap <<
                                        " same_gap: " << node_ref_->br->same_gap );
                if( node_ref_->br->next->type == END )
                        node_ref_ = NULL;
                else node_ref_ = node_ref_->br->next;
        }
}

int inter_det::TopoFeature::identifyIntersection()
{
	ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Method entered!" );
	if( node_head_ == NULL )
	{
		ROS_INFO_STREAM( "TopoFeature::identifyIntersection: No tree to traverse!" );
		return UNKNOWN;
	}
	else
	{
		ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Head type: " <<
				node_head_->type);
	}

	struct node *right, *left;
	int forw_entry, back_entry;
	right = node_head_;
	left = node_head_->br->next;

	ROS_INFO_STREAM( "TopoFeature::identifyIntersection: right type: " <<
				node_head_->type << "left type: " << node_head_->br->next->type );

	 ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Right Line: <" <<
                                        right->info.beg_x << "," << right->info.beg_y << "><" <<
                                        right->info.end_x << "," << right->info.end_y << ">" <<
					right->fr->rel_to );

	  ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Left Line: <" <<
                                        left->info.beg_x << "," << left->info.beg_y << "><" <<
                                        left->info.end_x << "," << left->info.end_y << ">" <<
					left->br->rel_to );


	bool left_space, right_space, center_space;
	left_space = right_space = center_space = false;

	while ( right != left )
	{
		ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Inside while loop!" );

		if ( right->fr->rel_to == FRONT || right->fr->rel_to == BACK ) 
		{
			ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Processed right forw Line: <" <<
					right->info.beg_x << "," << right->info.beg_y << "><" <<
					right->info.end_x << "," << right->info.end_y << ">" );
			if(right->fr->same_gap)
			{
				ROS_INFO_STREAM ( "TopoFeature::identifyIntersection: Set right space" );
				right_space = true;
			}
			right = right->fr->next;
		}
		else if ( right->fr->rel_to == LEFT )
		{
			ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Processed right side Line: <" <<
                                        right->info.beg_x << "," << right->info.beg_y << "><" <<
                                        right->info.end_x << "," << right->info.end_y << ">" );
			if (right->fr->next == left)
			{
				if( center_space )
				{
					ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Found left and sreturning with intersection!" );
					return(logIntersection(right_space, left_space, center_space));
				}
				else
				{
					if(right->fr->side_gap)
					{
						ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Found left!, Not yet returning!");
						center_space = true;
					}
				}
			}
		}

		if ( left->br->rel_to == BACK || left->br->rel_to == FRONT )
		{
			ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Processed left forw Line: <" <<
                                        left->info.beg_x << "," << left->info.beg_y << "><" <<
                                        left->info.end_x << "," << left->info.end_y << ">" );
			if(left->br->same_gap)
			{
				ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Setting left space" );
				left_space = true;
			}
			left = left->br->next;
		}
		else if ( left->br->rel_to == RIGHT )
		{
			ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Processed left side Line: <" <<
                                        left->info.beg_x << "," << left->info.beg_y << "><" <<
                                        left->info.end_x << "," << left->info.end_y << ">" );
			if (left->br->next == right)
                        {
                                if( center_space )
                                {
                                        ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Found right and sreturning with intersection!" );
                                        return(logIntersection(right_space, left_space, center_space));
                                }
                                else
                                {
                                        if(left->br->side_gap)
                                        {
                                                ROS_INFO_STREAM( "TopoFeature::identifyIntersection: Found right!, Not yet returning!");
                                                center_space = true;
                                        }
                                }
                        }
		}
	}
	return(logIntersection(right_space, left_space, center_space));
}

int inter_det::TopoFeature::logIntersection(bool right_space, bool left_space, bool center_space)
{
	if(left_space && right_space && center_space)
        {
        	ROS_INFO_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: FOUR_WAY" );
                return FOUR;
        }
        else if(left_space && right_space)
        {
        	ROS_INFO_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: T");
                return T;
        }
	else if(left_space)
        {
        	ROS_INFO_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: Left" );
                return LEFT;
        }
        else if(right_space)
        {
        	ROS_INFO_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: RIGHT" );
                return RIGHT;
        }
        else
        {
        	ROS_INFO_STREAM( "TopoFeature::identifyIntersection: INTERSECTION: NOTHING" );
                return UNKNOWN;
        }
}