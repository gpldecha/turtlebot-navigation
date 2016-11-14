#include <ros/ros.h>


#include "optitrack_rviz/input.h"

#include "agent/agent.h"

void occupancygrid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std::cout<< "occupancygrid_callback !! " << std::endl;
}

int main(int argc, char** argv)
{


    // -------------- Get node input paramters --------------

    std::map<std::string,std::string> input;
    input["-occupancy_topic"]   = "/move_base/global_costmap/costmap_updates";
    input["-fixed_frame"]       = "/world_frame";
    input["-rate"]              = "30";

    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    double      rate_hz               = boost::lexical_cast<double>(input["-rate"]);
    std::string fixed_frame           = input["-fixed_frame"];
    std::string occupancy_grid_topic  = "/move_base/global_costmap/costmap";//input["-occupancy_topic"];

    // ----------------- INITIALISE NODE ----------------- //

    ros::init(argc, argv, "topology_map");
    ros::NodeHandle nh;

    // ----------------- SETUP DISCRETE STATES AND ADJACENCY MATRIX ----------------- //

    ROS_INFO("Initialise Occupancy Grid Map Listener");
    // c2t::OccupancyGridListener occupancyGridListener(occupancy_grid_topic,nh);
    //occupancyGridListener.wait_msg();
    //nh.subscribe(topic_name, 10, &OccupancyGridListener::occupancygrid_callback, this);

    ros::Subscriber sub = nh.subscribe("/map",100,occupancygrid_callback);

    arma::mat grid;
    topolog_map::create_2d_grid(grid,0,0,10,10,10,10);

    ROS_INFO("Grids Initalised!");


    // ----------------- OBJECTIVE FUNCTION ----------------- //

    arma::ucolvec occupancy(grid.n_rows,arma::fill::zeros);
    search::objective_func obj_func =  std::bind(search::objective_function_one,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3,
                                                 std::ref(occupancy),
                                                 std::ref(grid));

    // ----------------- AGENTS ----------------- //

    arma::colvec3 agent_pos_3D = {grid(0,0),grid(0,1),0};

    search::Agent agent(std::string("agent1"),nh,grid,agent_pos_3D,obj_func,occupancy);
    agent.initialise_visualisation(nh,fixed_frame);
    agent.start();


    ros::Time now;
    ros::Rate rate(60);
    while(nh.ok()){

        now    = ros::Time::now();

        agent.update(now);

        agent.publish();


        ros::spinOnce();
        rate.sleep();
    }




    return 0;
}
