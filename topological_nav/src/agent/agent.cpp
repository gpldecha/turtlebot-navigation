#include "agent/agent.h"

namespace search{


Agent::Agent(const std::string &name, ros::NodeHandle& nh, const arma::mat& grid, const arma::colvec3 &agent_pos_3D, objective_func &obj_func, arma::ucolvec &occupancy):
  name(name),
  grid(grid),
  agent_pos_3D(agent_pos_3D),
  occupancy(occupancy)
{

    bVisionInit     = false;
    bRun            = false;
    current_state   = 0;
    last_state      = 0;
    target_state    = 0;

    topolog_map::TopologyMap::build_adjacency_matrix(Adj,grid,1);

    Adj.print("Adj");

    online_search_ptr.reset(new search::Online_search(Adj,obj_func));

    double dist_threashold  = 0.1;
    double check_rate       = 1;
    fsm_ptr.reset( new search::FSM(grid,dist_threashold,check_rate,ros::Time::now()));

  //  service                 = nh.advertiseService(service_name,&Cmd_interface::service_callback,this);

    service = nh.advertiseService(name + "_cmd", &Agent::service_callback,this);
}



void Agent::start(){

    agent_pos_2D    = agent_pos_3D(arma::span(0,1)).st();

    period          = 1.0/60.0;

    current_state   = search::get_state(agent_pos_2D,grid);
    target_state    = current_state;
    last_state      = current_state;

    fsm_ptr->set_target(target_state);
}

void Agent::update(const ros::Time& now){

    if(!bRun)
        return;

    agent_pos_2D = agent_pos_3D(arma::span(0,1)).st();

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

        /*std::cout<< "current_state: " << current_state << std::endl;
        std::cout<< "last_state: " << last_state << std::endl;
        std::cout<< "Adj:       " << Adj.n_rows << " x " << Adj.n_cols << std::endl;
        std::cout<< "grid:      " << grid.n_rows << " x " << grid.n_cols << std::endl;
        std::cout<< "occupancy: " << occupancy.n_elem << std::endl;*/


        search::action a = online_search_ptr->get_action(current_state,last_state,1);

        target_state     = search::forward_dynamics(current_state,a,Adj);


        fsm_ptr->set_target(target_state);

        target_pos_3D(arma::span(0,1)) = grid.row(target_state).st();
        search::set_color(occupancy,colors);

    }

    search::get_velocity(velocity,agent_pos_3D,target_pos_3D,max_speed,period);

    agent_pos_3D = agent_pos_3D + velocity;

}


void Agent::publish(){

    if(!bVisionInit)
        ROS_WARN_STREAM_THROTTLE(1.0,"Agent::initialise_visualisation has not been called!");

    // update visualisation

    agent_pos_vis[0].setX(agent_pos_3D(0));
    agent_pos_vis[0].setY(agent_pos_3D(1));
    agent_pos_vis[0].setZ(agent_pos_3D(2));
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

void Agent::initialise_visualisation(ros::NodeHandle &nh, const std::string& fixed_frame){


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
    agent_pos_vis[0] = tf::Vector3(agent_pos_3D(0),agent_pos_3D(1),agent_pos_3D(2));

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

bool Agent::has_finished_task(){
    for(int i = 0; i < occupancy.n_elem;i++)
    {
        if(occupancy(i) == 0)
            return false;
    }
    return true;
}


bool Agent::service_callback(topological_nav::String_cmd::Request& req, topological_nav::String_cmd::Response& res){

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
