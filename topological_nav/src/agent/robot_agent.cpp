#include "agent/robot_agent.h"

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

    topolog_map::build_adjacency_matrix(Adj,grid,1);

    online_search_ptr.reset(new search::Online_search(Adj,obj_func));

    double dist_threashold  = 0.1;
    double check_rate       = 1;
    fsm_ptr.reset( new search::FSM(grid,dist_threashold,check_rate,ros::Time::now()));

    service = nh.advertiseService(name + "_cmd", &RobotAgent::service_callback,this);



    //tell the action client that we want to spin a thread by default

    move_base_client_ptr.reset( new MoveBaseClient("move_base", true) );


    //wait for the action server to come up
    while(!move_base_client_ptr->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }


    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";


    /*
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the base moved 1 meter forward");
    }else{
      ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
    */



}



void RobotAgent::start(const arma::colvec3 &robot_position){


    agent_pos_2D        = robot_position(arma::span(0,1)).st();
    agent_sim_pos_3D    = robot_position;

    period              = 1.0/60.0;

    current_state       = search::get_state(agent_pos_2D,grid);
    target_state        = current_state;
    last_state          = current_state;

    fsm_ptr->set_target(target_state);
}

void RobotAgent::update(const ros::Time& now, const arma::colvec3& robot_position){



    if(!bRun)
        return;

    agent_pos_2D(0) = robot_position(0);
    agent_pos_2D(1) = robot_position(1);

    //agent_pos_2D.print("agent_pos_2D");

    if(fsm_ptr->has_reached_target(agent_pos_2D,now))
    {
        ROS_INFO_STREAM("Has reached target");

        occupancy(fsm_ptr->get_target()) = 1;

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

        target_pos_3D(arma::span(0,1)) = grid.row(target_state).st();
        search::set_color(occupancy,colors);

        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x    = target_pos_3D(0);
        goal.target_pose.pose.position.y    = target_pos_3D(1);

        goal.target_pose.pose.orientation.w = 1.0;

        ROS_INFO("Sending goal");
        move_base_client_ptr->sendGoal(goal);
    }

    search::get_velocity(velocity,agent_sim_pos_3D,target_pos_3D,max_speed,period);
    agent_sim_pos_3D = agent_sim_pos_3D + velocity;

}


void RobotAgent::publish(){

    if(!bVisionInit)
        ROS_WARN_STREAM_THROTTLE(1.0,"Agent::initialise_visualisation has not been called!");

    // update visualisation

    agent_pos_vis[0].setX(agent_sim_pos_3D(0));
    agent_pos_vis[0].setY(agent_sim_pos_3D(1));
    agent_pos_vis[0].setZ(agent_sim_pos_3D(2));
    vis_point_ptr->update(agent_pos_vis);

    // plot grid
    vis_grid_ptr->update(colors);
    vis_grid_ptr->publish();


    // plot agent
    vis_point_ptr->publish();

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
    agent_pos_vis.resize(1);
    agent_pos_vis[0] = tf::Vector3(agent_sim_pos_3D(0),agent_sim_pos_3D(1),agent_sim_pos_3D(2));

    vis_point_ptr.reset( new opti_rviz::Vis_points(nh,name) );

    vis_point_ptr->r = 1; vis_point_ptr->g = 1; vis_point_ptr->b = 1;
    vis_point_ptr->scale = 0.15;
    vis_point_ptr->initialise(fixed_frame,agent_pos_vis);

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
