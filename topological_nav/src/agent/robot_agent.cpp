#include "agent/robot_agent.h"
#include "topological_map/topologymap.h"

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>


namespace search{


RobotAgent::RobotAgent(const std::string &name, ros::NodeHandle& nh, const arma::mat& grid, objective_func &obj_func, arma::ucolvec &occupancy):
  name(name),
  grid(grid),
  occupancy(occupancy)
{

    bVisionInit     = false;
    bRun            = false;
    current_state   = 0;
    last_state      = 0;
    target_state    = 0;

    topolog_map::TopologyMap::build_adjacency_matrix(Adj,grid,1);

    online_search_ptr.reset(new search::Online_search(Adj,obj_func));

    double dist_threashold  = 0.5;
    double check_rate       = 1;
    fsm_ptr.reset( new search::FSM(grid,dist_threashold,check_rate,ros::Time::now()));

    service = nh.advertiseService(name + "_cmd", &RobotAgent::service_callback,this);

    pub = nh.advertise<geometry_msgs::Pose2D>("goal_sweeper", 100);





    //tell the action client that we want to spin a thread by default

    move_base_client_ptr.reset( new MoveBaseClient("move_base", true) );


}



void RobotAgent::start(const arma::colvec3 &robot_position){
    ROS_INFO("RobotAgent::start (start)");

    agent_pos_2D        = robot_position(arma::span(0,1)).st();
    agent_sim_pos_3D    = robot_position;

    period              = 1.0/60.0;

    current_state       = search::get_state(agent_pos_2D,grid);
    last_state          = current_state;

    // find closet point
    std::size_t idx;
    topolog_map::TopologyMap::closest_point(agent_pos_2D,grid,idx);

    target_state                   = idx;
    target_pos_2D                  = grid.row(target_state);

    target_pos_3D(0)               = target_pos_2D(0);
    target_pos_3D(1)               = target_pos_2D(1);


    count_goals                    = 0;

    fsm_ptr->set_target(target_state);
    ROS_INFO("RobotAgent::start (end)");
    bMovingRobot = false;

}

void RobotAgent::send_robot_goal(arma::rowvec2 coordinate,const double target_yaw){

    ROS_INFO_STREAM("DEBUG    SEND_ROBOT_GOAL (start)");


    move_base_msgs::MoveBaseGoal          goal;
    goal.target_pose.header.frame_id    = "base_link";
    goal.target_pose.header.stamp       = ros::Time::now();
    goal.target_pose.pose.position.x    = coordinate(0);
    goal.target_pose.pose.position.y    = coordinate(1);

    ROS_INFO_STREAM("DEBUG   TARGET " << coordinate(0) << " " << coordinate(1) );


    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);
    goal.target_pose.pose.orientation.x = goal_quat.x();
    goal.target_pose.pose.orientation.y = goal_quat.y();
    goal.target_pose.pose.orientation.z = goal_quat.z();
    goal.target_pose.pose.orientation.w = goal_quat.w();


    actionlib::SimpleClientGoalState astate = move_base_client_ptr->getState();
    typedef actionlib::SimpleClientGoalState::StateEnum StateEnum;

    if(astate.state_ != StateEnum::SUCCEEDED)
    {

        move_base_client_ptr->cancelGoal();

        bool finished_before_timeout = move_base_client_ptr->waitForResult(ros::Duration(5.0));

         if (finished_before_timeout)
         {
           astate = move_base_client_ptr->getState();
           ROS_INFO("Action finished: %s",astate.toString().c_str());
         }
         else
           ROS_INFO("Action did not finish before the time out.");

    }


    //goal.target_pose.pose.orientation.w = angle;
    ROS_INFO("Sending goal");
    move_base_client_ptr->sendGoal(goal);

    geometry_msgs::Pose2D pose_2D_msg;
    pose_2D_msg.x = coordinate(0);
    pose_2D_msg.y = coordinate(1);

    pub.publish(pose_2D_msg);


   ROS_INFO_STREAM("DEBUG    SEND_ROBOT_GOAL (end)");
}

void RobotAgent::update(const ros::Time& now, const arma::colvec3& robot_position){


    if(!bRun)
        return;

    agent_pos_2D(0) = robot_position(0);
    agent_pos_2D(1) = robot_position(1);

    actionlib::SimpleClientGoalState astate = move_base_client_ptr->getState();

     ROS_INFO_STREAM_THROTTLE(1.0,"ActionServer state: " << astate.toString());
    //agent_pos_2D.print("agent_pos_2D");

    if(fsm_ptr->has_reached_target(agent_pos_2D,now))
    {
        ROS_INFO_STREAM("Has reached target:    " << count_goals);

        occupancy(fsm_ptr->get_target()) = 1;
        count_goals++;

        if(has_finished_task()){
            bRun = false;
            ROS_INFO_STREAM("..........Task complete!");
            return;
        }

        last_state       = current_state;
        current_state    = search::get_state(agent_pos_2D,grid);

        search::action a = online_search_ptr->get_action(current_state,last_state,1);

        target_state     = search::forward_dynamics(current_state,a,Adj);


        fsm_ptr->set_target(target_state);

        target_pos_2D                  = grid.row(target_state);
        target_pos_3D(0)               = target_pos_2D(0);
        target_pos_3D(1)               = target_pos_2D(1);

        search::set_color(occupancy,colors);

        send_robot_goal(target_pos_2D,0);
        bMovingRobot = true;

    }else if(!bMovingRobot){
        send_robot_goal(target_pos_2D,0);
        bMovingRobot = true;
    }

    search::get_velocity(velocity,agent_sim_pos_3D,target_pos_3D,max_speed,period);
    agent_sim_pos_3D = agent_sim_pos_3D + velocity;

}


void RobotAgent::publish(){

    if(!bVisionInit)
        ROS_WARN_STREAM_THROTTLE(1.0,"Agent::initialise_visualisation has not been called!");

    // update visualisation

    vis_agent_point[0].setX(agent_sim_pos_3D(0));
    vis_agent_point[0].setY(agent_sim_pos_3D(1));
    vis_agent_point[0].setZ(agent_sim_pos_3D(2));

    vis_target_point[0].setX(target_pos_3D(0));
    vis_target_point[0].setY(target_pos_3D(1));
    vis_target_point[0].setZ(target_pos_3D(2));


    vis_agent_pos_ptr->update(vis_agent_point);

    vis_target_pos_ptr->update(vis_target_point);


    // plot grid
    vis_grid_ptr->update(colors);
    vis_grid_ptr->publish();


    // plot agent
    vis_agent_pos_ptr->publish();

    // plot target
    vis_target_pos_ptr->publish();


    // plot neighbours

    search::get_neighbours(neighbours,current_state,Adj,grid);
    vis_neighbours_ptr->update_dynamic(neighbours);
    vis_neighbours_ptr->publish();

}


void RobotAgent::initialise_visualisation(ros::NodeHandle &nh, const std::string& fixed_frame){


    // visualise the grid
    vis_grid_ptr.reset( new opti_rviz::Vis_gird(nh,name + "_grid_vis") );
    vis_grid_ptr->scale = 0.1;
    vis_grid_ptr->default_z = 0;
    vis_grid_ptr->initialise(fixed_frame,grid);

    colors.resize(grid.n_rows);
    std::array<float,4> color = {{1,1,0,0}};
    opti_rviz::set_all_colors(colors,color);

    // visualise the agent
    vis_agent_point.resize(1);
    vis_agent_point[0] = tf::Vector3(agent_sim_pos_3D(0),agent_sim_pos_3D(1),agent_sim_pos_3D(2));


    vis_agent_pos_ptr.reset( new opti_rviz::Vis_points(nh,name) );

    vis_agent_pos_ptr->r = 1; vis_agent_pos_ptr->g = 1; vis_agent_pos_ptr->b = 1;
    vis_agent_pos_ptr->scale = 0.15;
    vis_agent_pos_ptr->initialise(fixed_frame,vis_agent_point);

    vis_target_point.resize(1);
    vis_target_point[0] = tf::Vector3(target_pos_3D(0),target_pos_3D(1),target_pos_3D(2));

    vis_target_pos_ptr.reset( new opti_rviz::Vis_points(nh,name + "_target") );
    vis_target_pos_ptr->r = 0.5; vis_agent_pos_ptr->g = 0.5; vis_agent_pos_ptr->b = 1;
    vis_target_pos_ptr->scale = 0.2;
    vis_target_pos_ptr->initialise(fixed_frame,vis_target_point);

    //visualise neighbours
    search::get_neighbours(neighbours,current_state,Adj,grid);

    vis_neighbours_ptr.reset( new opti_rviz::Vis_points(nh,name + "_neighbours") );
    vis_neighbours_ptr->r = 1; vis_neighbours_ptr->g = 0; vis_neighbours_ptr->b = 1;
    vis_neighbours_ptr->scale = 0.15;
    vis_neighbours_ptr->initialise(fixed_frame,neighbours);

    bVisionInit = true;

}

bool RobotAgent::has_finished_task(){
    for(int i = 0; i < occupancy.n_elem;i++)
    {
        if(occupancy(i) == 0)
            return false;
    }
    return true;
}


bool RobotAgent::service_callback(topological_nav::String_cmd::Request& req, topological_nav::String_cmd::Response& res){

    std::string cmd = req.cmd;

    if(cmd == "start")
    {
        bRun = true;
        ROS_INFO("Starting Agent!");
    }else if(cmd == "stop")
    {
        bRun = false;
        ROS_INFO("Stopping Agent!");
    }else{
        ROS_WARN_STREAM("Agent::service_callback: No such cmd: " + cmd);
    }

}


}
