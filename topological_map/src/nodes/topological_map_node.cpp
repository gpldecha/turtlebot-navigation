#include <ros/ros.h>


#include "optitrack_rviz/input.h"
#include "visualise/vis_grid.h"

#include "topological_map/rviz_topology.h"
#include "topological_map/occupancygrid_listener.h"
#include "topological_map/costmap2topology.h"

#include "online_search/online_search.h"
#include "online_search/objective_functions.h"

#include "finite_state_machine/finite_state_machine.h"

#include "visualise/vis_points.h"


int main(int argc, char** argv)
{


    // -------------- Get node input paramters --------------

    std::map<std::string,std::string> input;
    input["-occupancy_topic"]   = "/world_frame";
    input["-fixed_frame"]       = "/world_frame";
    input["-rate"]              = "30";

    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    double      rate_hz               = boost::lexical_cast<double>(input["-rate"]);
    std::string fixed_frame           = input["-fixed_frame"];
    std::string occupancy_grid_topic  = input["-occupancy_topic"];

    // ----------------- INITIALISE NODE ----------------- //

    ros::init(argc, argv, "topology_map");
    ros::NodeHandle nh;

    // ----------------- SETUP DISCRETE STATES AND ADJACENCY MATRIX ----------------- //

   // c2t::OccupancyGridListener occupancyGridListener(occupancy_grid_topic,nh);
    //occupancyGridListener.wait_msg();

    arma::mat grid;
    topolog_map::create_2d_grid(grid,0,0,10,10,1);

    arma::Mat<int> Adj;
    topolog_map::build_adjacency_matrix(Adj,grid,1);

    Adj.print("Adj");


    // ----------------- INITIALISE AGENT POSITION ----------------- //

    arma::rowvec2 agent_pos_2D = {0,0};
    arma::colvec3 agent_pos_3D = {0,0,0};

    // ----------------- ONLINE SEARCH INTITIALISATION ----------------- //

    std::cout<< " Online search initialised " << std::endl;

    arma::ucolvec occupancy(Adj.n_rows,arma::fill::zeros);
    search::objective_func obj_func =  std::bind(search::objective_function_one,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3,
                                                 std::ref(occupancy),
                                                 std::ref(grid));

    search::Online_search online_search(Adj,obj_func);
    online_search.update_state(agent_pos_2D,grid);


    // ----------------- STATE TRACKER INTITIALISATION ----------------- //

    ros::Time time = ros::Time::now();


    search::state current_state = search::get_state(agent_pos_2D,grid);
    search::state target_state  = current_state;

    double dist_threashold  = 0.1;
    double check_rate       = 1;
    search::FSM fsm(grid,dist_threashold,check_rate,time);
    fsm.set_target(target_state);


    arma::colvec3 velocity;
    arma::colvec3 target_pos_3D;
    double max_speed = 0.2; // m/s

    ros::Time last, now;
    ros::Duration period;



    // ----------------- INITIALISE VISUALISATION ----------------- //


    // visualise the grid
    opti_rviz::Vis_gird vis_grid(nh,"grid_vis");
    vis_grid.scale = 0.1;
    vis_grid.default_z = 0;
    vis_grid.initialise(fixed_frame,grid);
    opti_rviz::acolor colors(grid.n_rows);
    std::array<float,4> one_color = {{1,1,0,0}};
    opti_rviz::set_all_colors(colors,one_color);


    // visualise the agent
    std::vector<tf::Vector3> agent_pos_vis(1);
    agent_pos_vis[0] = tf::Vector3(agent_pos_2D(0),agent_pos_2D(1),0.1);
    opti_rviz::Vis_points   vis_point(nh,"agent1");
    vis_point.r = 1; vis_point.g = 1; vis_point.b = 1;
    vis_point.scale = 0.15;
    vis_point.initialise(fixed_frame,agent_pos_vis);

    //visualise adjacency matrix
    std::vector<tf::Vector3> neighbours;
    search::get_neighbours(neighbours,current_state,Adj,grid);


    opti_rviz::Vis_points   vis_point_adj(nh,"neighbours");
    vis_point_adj.r = 1; vis_point_adj.g = 0; vis_point_adj.b = 1;
    vis_point_adj.scale = 0.15;
    vis_point_adj.initialise(fixed_frame,neighbours);


     ros::Rate rate(60);
     while(nh.ok()){

         now    = ros::Time::now();
         period = now - last;

         agent_pos_2D = agent_pos_3D(arma::span(0,1)).st();


         if(fsm.has_reached_target(agent_pos_2D,now))
         {
             ROS_INFO_STREAM("Has reached target");

             occupancy(fsm.get_target()) = 1;

             search::action a = online_search.get_action(1);

             current_state    = search::get_state(agent_pos_2D,grid);

             target_state     = search::forward_dynamics(current_state,a,Adj);

             fsm.set_target(target_state);

             target_pos_3D(arma::span(0,1)) = grid.row(target_state).st();

             search::set_color(occupancy,colors);

         }

         // apply velocity to agent position

         search::get_velocity(velocity,agent_pos_3D,target_pos_3D,max_speed,period);

         agent_pos_3D = agent_pos_3D + velocity;


         // update visualisation

         agent_pos_vis[0].setX(agent_pos_3D(0));
         agent_pos_vis[0].setY(agent_pos_3D(1));
         agent_pos_vis[0].setZ(agent_pos_3D(2));
         vis_point.update(agent_pos_vis);

         // plot grid
         vis_grid.update(colors);
         vis_grid.publish();


         // plot agent
         vis_point.publish();

         // plot neighbours
         vis_point_adj.publish();

         ros::spinOnce();
         rate.sleep();
         last = now;
     }




	return 0;
}
