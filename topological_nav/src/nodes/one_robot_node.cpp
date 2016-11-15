#include <ros/ros.h>


#include "optitrack_rviz/input.h"
#include "topological_map/vis_map.h"

#include "topological_map/costmap2topology.h"

#include "agent/agent.h"
#include "agent/robot_agent.h"

#include <optitrack_rviz/debug.h>
#include <math.h>

#include <optitrack_rviz/listener.h>



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
    std::string occupancy_grid_topic  = "/map";//input["-occupancy_topic"];

    // ----------------- INITIALISE NODE ----------------- //

    ros::init(argc, argv, "topology_map");
    ros::NodeHandle nh;

    // ----------------- SETUP DISCRETE STATES AND ADJACENCY MATRIX ----------------- //

    ROS_INFO("Initialise Occupancy Grid Map Listener");
    c2t::OccupancyGridListener occupancyGridListener;
    ros::Subscriber sub = nh.subscribe(std::string("/map"),100,&c2t::OccupancyGridListener::occupancygrid_callback,&occupancyGridListener);
    occupancyGridListener.wait_msg(nh);

    grid_map::GridMap& map = occupancyGridListener.get_grid_map();


    /// CREATE TOPOLOGY NETWORK

    c2t::OccupancyMap2Topology occ2t(map);
    arma::colvec3 T;
    arma::mat33 Rot;
    occ2t.get_frame_reference(T,Rot);
    occ2t.get_bbox();



    std::string map_topic_name = "vis_map";
    topolog_map::Vis_map vis_map(map,occ2t.g_points,occ2t.b_points,map_topic_name,fixed_frame,nh);


    //8,6

    arma::mat grid;
    topolog_map::TopologyMap::create_2d_grid(grid,T(0) + 0.45,T(1) + 0.3,6,4.5,6,3);
    // (N x 2)
    arma::mat22 RRot;
    double angle = 9 * M_PI / 180.0;
    RRot(0,0) = cos(angle);
    RRot(0,1) = -sin(angle);
    RRot(1,0) =  sin(angle);
    RRot(1,1) = cos(angle);
    RRot.print("RRot");

    grid = (RRot * grid.st()).st();

    //occ2t.adapt_grid(grid);



    ROS_INFO("Grids Initalised!");



    // ----------------- OBJECTIVE FUNCTION ----------------- //

    arma::ucolvec occupancy(grid.n_rows,arma::fill::zeros);
    search::objective_func obj_func =  std::bind(search::objective_function_one,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3,
                                                 std::ref(occupancy),
                                                 std::ref(grid));


    // ----------------- Robot Position Listener ----------------- //



    opti_rviz::Listener listener(fixed_frame,"base_link");
    tf::Vector3     robot_pos;
    tf::Matrix3x3   robot_orient;
    arma::colvec3  robot_pos_mat;

    bool bReceivedMessage = false;
    ros::Rate rate1(10);
    int count = 0;
    while(nh.ok() && bReceivedMessage == false)
    {
        bReceivedMessage = listener.update(robot_pos,robot_orient);

        ros::spinOnce();
        rate1.sleep();
    }

    ROS_INFO("   Got Robot Pos");



    opti_rviz::type_conv::tf2vec(robot_pos,robot_pos_mat);

    // ----------------- AGENTS ----------------- //

    // get position of agent



    search::RobotAgent robot_agent(std::string("agent1"),nh,grid,obj_func,occupancy);
    robot_agent.initialise_visualisation(nh,fixed_frame);
    robot_agent.start(robot_pos_mat);

    arma::colvec3 agent_pos_3D = {grid(0,0),grid(0,1),0};
    search::Agent agent(std::string("agent2"),nh,grid,agent_pos_3D,obj_func,occupancy);
    agent.initialise_visualisation(nh,fixed_frame);
    agent.start();





    ROS_INFO_STREAM("start while loop");
    ros::Time now;
    ros::Rate rate(60);
    while(nh.ok()){

        now    = ros::Time::now();

        listener.update(robot_pos,robot_orient);

        opti_rviz::type_conv::tf2vec(robot_pos,robot_pos_mat);

        vis_map.publish();

        //opti_rviz::debug::tf_debuf(T,Rot,"FRAME OF REFERENCE",fixed_frame);
       // opti_rviz::debug::tf_debuf(occ2t.tt,"tt",fixed_frame);


        robot_agent.update(now,robot_pos_mat);
        robot_agent.publish();


        ros::spinOnce();
        rate.sleep();
    }




    return 0;
}
