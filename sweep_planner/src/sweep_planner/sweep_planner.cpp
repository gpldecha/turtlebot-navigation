#include <pluginlib/class_list_macros.h>
#include "sweep_planner/sweep_planner.h"

PLUGINLIB_EXPORT_CLASS(global_planner::Sweep, nav_core::BaseGlobalPlanner);

namespace global_planner {

Sweep::Sweep():costmap_ros_(NULL), initialized_(false){}

Sweep::Sweep(std::string name, costmap_2d::Costmap2DROS* costmap_ros):
costmap_ros_(NULL), initialized_(false)
{
    initialize(name, costmap_ros);
}


void Sweep::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_){


        costmap_ros_    = costmap_ros;
        costmap_        = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
        world_model_    = new base_local_planner::CostmapModel(*costmap_);


      /*  costMap2Top_sptr.reset( new c2t::CostMap2Topology(*costmap_));*/



       initialized_     = true;
     }else{
           ROS_WARN("This planner has already been initialized... doing nothing");
    }

    speed = 0;
}


//we need to take the footprint of the robot into account when we calculate cost to obstacles
double Sweep::footprintCost(double x_i, double y_i, double theta_i){

    if(!initialized_){
       ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
       return -1.0;
     }

     std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
     //if we have no footprint... do nothing
     if(footprint.size() < 3)
       return -1.0;

     //check if the footprint is legal
     double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
     return footprint_cost;
}

bool Sweep::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

     if(!initialized_){
       ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
       return false;
     }

   /* vis_grid_ptr->publish();
     ros::spinOnce();*/


     ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

     plan.clear();
     costmap_ = costmap_ros_->getCostmap();

     if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
       ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
           costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
       return false;
     }

     tf::Stamped<tf::Pose> goal_tf;
     tf::Stamped<tf::Pose> start_tf;

     poseStampedMsgToTF(goal,goal_tf);
     poseStampedMsgToTF(start,start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
     start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
     goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

     //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
     double goal_x = goal.pose.position.x;
     double goal_y = goal.pose.position.y;
     double start_x = start.pose.position.x;
     double start_y = start.pose.position.y;

     double diff_x = goal_x - start_x;
     double diff_y = goal_y - start_y;
     double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

     double target_x = goal_x;
     double target_y = goal_y;
     double target_yaw = goal_yaw;

     bool done = false;
     double scale = 1.0;
     double dScale = 0.01;

     while(!done)
     {
       if(scale < 0)
       {
         target_x = start_x;
         target_y = start_y;
         target_yaw = start_yaw;
         ROS_WARN("The carrot planner could not find a valid plan for this goal");
         break;
       }
       target_x = start_x + scale * diff_x;
       target_y = start_y + scale * diff_y;
       target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

       double footprint_cost = footprintCost(target_x, target_y, target_yaw);
       if(footprint_cost >= 0)
       {
           done = true;
       }
       scale -=dScale;
     }

     plan.push_back(start);
     geometry_msgs::PoseStamped new_goal = goal;
     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

     new_goal.pose.position.x = target_x;
     new_goal.pose.position.y = target_y;

     new_goal.pose.orientation.x = goal_quat.x();
     new_goal.pose.orientation.y = goal_quat.y();
     new_goal.pose.orientation.z = goal_quat.z();
     new_goal.pose.orientation.w = goal_quat.w();

     plan.push_back(new_goal);
     return (done);
}

/*
bool Sweep::makePlan(   const geometry_msgs::PoseStamped&                   start,
                        const geometry_msgs::PoseStamped&                   goal,
                               std::vector<geometry_msgs::PoseStamped>&     plan
             )
{
   ROS_INFO_STREAM_THROTTLE(1.0,"----------> Sweep making plan ");
   if(!initialized_){
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return false;
   }

   plan.clear();
   costmap_ = costmap_ros_->getCostmap();

   plan.push_back(start);

   speed = (1.0/20.0);

   current_pos = {start.pose.position.x,start.pose.position.y};
   target_pos  = {goal.pose.position.x, goal.pose.position.y};

   //current_pos.setValue(start.pose.position.x,start.pose.position.y,start.pose.position.z);


   for (int i=0; i<20; i++){
        geometry_msgs::PoseStamped new_goal = goal;
        //tf::Quaternion goal_quat = tf::createQuaternionFromYaw(start.pose.orientation);



        // go in a straight line towards the goal


        next_pos = speed * (target_pos - current_pos) + current_pos;
        current_pos = next_pos;

        new_goal.pose.position.x =  next_pos(0);
        new_goal.pose.position.y =  next_pos(1);
        new_goal.pose.orientation = start.pose.orientation;

       /* new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();*/
/*
        plan.push_back(new_goal);
   }

    plan.push_back(goal);
    return true;
}
*/

}
