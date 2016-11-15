#ifndef TOPOLOGY_ROBOT_AGENT_H_
#define TOPOLOGY_ROBOT_AGENT_H_

#include <armadillo>
#include <visualise/vis_grid.h>
#include <visualise/vis_points.h>

#include "topological_map/rviz_topology.h"
#include "topological_map/occupancygrid_listener.h"
#include "topological_map/costmap2topology.h"

#include "online_search/online_search.h"
#include "online_search/objective_functions.h"
#include "finite_state_machine/finite_state_machine.h"

#include "topological_nav/String_cmd.h"

#include <vector>

#include <tf/LinearMath/Vector3.h>


#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


namespace search{

class RobotAgent{

public:

    RobotAgent(const std::string&  name,
               ros::NodeHandle&    nh,
         const arma::mat&          grid,
               objective_func&     obj_func,
               arma::ucolvec&      occupancy);

   void start(const arma::colvec3& robot_position);

   void update(const ros::Time &now, const arma::colvec3& robot_position);

   void publish();

   void initialise_visualisation(ros::NodeHandle& nh, const std::string &fixed_frame);

//   void initialise_robot_pos_listener(const std::string &fixed_frame, const std::string& target_frame="base_link");

   bool has_finished_task();

   void send_robot_goal(arma::rowvec2 coordinate, const double target_yaw);

private:



    bool service_callback(topological_nav::String_cmd::Request& req, topological_nav::String_cmd::Response& res);

private:

    const arma::mat&                               grid;
    arma::ucolvec&                                 occupancy;
    std::string                                    name;

    arma::Mat<int>                                 Adj;
    state                                          current_state, target_state, last_state;

    bool                                           bVisionInit;

    double                                         max_speed = 0.2; // m/s
    double                                         period;
    arma::colvec3                                  velocity;
    arma::colvec3                                  agent_sim_pos_3D;

    arma::rowvec2                                  agent_pos_2D;
    arma::colvec3                                  target_pos_3D;
    arma::rowvec2                                  target_pos_2D;

    std::unique_ptr<search::FSM>                   fsm_ptr;
    std::unique_ptr<search::Online_search>         online_search_ptr;

    std::unique_ptr<opti_rviz::Vis_gird>           vis_grid_ptr;
    std::unique_ptr<opti_rviz::Vis_points>         vis_agent_pos_ptr;
    std::unique_ptr<opti_rviz::Vis_points>         vis_target_pos_ptr;

    std::unique_ptr< opti_rviz::Vis_points>        vis_neighbours_ptr;

    std::vector<tf::Vector3>                       vis_agent_point;
    std::vector<tf::Vector3>                       vis_target_point;


    std::vector<tf::Vector3>                       neighbours;
    opti_rviz::acolor                              colors;

    bool                                           bRun;


    ros::ServiceServer                             service;

    std::unique_ptr<MoveBaseClient>                move_base_client_ptr;

    ros::Publisher                                 pub;

    bool                                           bMovingRobot;
    std::size_t                                    count_goals;


};

}

#endif
