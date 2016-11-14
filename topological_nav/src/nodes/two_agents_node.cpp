#include <ros/ros.h>


#include "optitrack_rviz/input.h"

#include "agent/agent.h"


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

    ros::init(argc, argv, "two_agents");
    ros::NodeHandle nh;


    // ----------------- SETUP DISCRETE STATES AND ADJACENCY MATRIX ----------------- //

    arma::mat grid;
    topolog_map::create_2d_grid(grid,0,0,10,10,10,10);

    std::vector<arma::mat> grids(2);
    topolog_map::partition_space(grid,grids,0);

    std::cout<< "grids[0]: " << grids[0].n_rows << " x " << grids[0].n_cols << std::endl;
    std::cout<< "grids[1]: " << grids[1].n_rows << " x " << grids[1].n_cols << std::endl;

    ROS_INFO("Grids Initalised!");

    // ----------------- OBJECTIVE FUNCTION ----------------- //

    // FOR AGENT 1
    arma::ucolvec occupancy_1(grids[0].n_rows,arma::fill::zeros);
    search::objective_func obj_func_1 =std::bind(search::objective_function_one,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3,
                                                 std::ref(occupancy_1),
                                                 std::ref(grids[0]));
    // FOR AGENT 2
    arma::ucolvec occupancy_2(grids[1].n_rows,arma::fill::zeros);
    search::objective_func obj_func_2 =std::bind(search::objective_function_one,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3,
                                                 std::ref(occupancy_2),
                                                 std::ref(grids[1]));

    // ----------------- AGENTS ----------------- //


    // AGENT 1
    arma::colvec3 agent1_pos_3D = {grids[0](0,0),grids[0](0,1),0};
    search::Agent agent1(std::string("agent1"),nh,grids[0],agent1_pos_3D,obj_func_1,occupancy_1);
    agent1.initialise_visualisation(nh,fixed_frame);
    agent1.start();

    // AGENT 2
    arma::colvec3 agent2_pos_3D = {grids[1](0,0),grids[1](0,1),0};
    search::Agent agent2(std::string("agent2"),nh,grids[1],agent2_pos_3D,obj_func_2,occupancy_2);
    agent2.initialise_visualisation(nh,fixed_frame);
    agent2.start();

    ros::Time now;
    ros::Rate rate(60);
    while(nh.ok()){


        now    = ros::Time::now();

        agent1.update(now);
        agent2.update(now);

        agent1.publish();
        agent2.publish();

        ros::spinOnce();
        rate.sleep();

    }


    return 0;
}
