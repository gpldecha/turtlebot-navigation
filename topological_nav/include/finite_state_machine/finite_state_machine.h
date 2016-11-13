#ifndef TOPOLOGY_FINITE_STATE_MACHINE_H_
#define TOPOLOGY_FINITE_STATE_MACHINE_H_

#include <ros/ros.h>

#include "online_search/utils.h"
#include <armadillo>

namespace search{

class FSM{

public:

    FSM(const arma::mat& grid,double dist_threashold,double publish_rate_, const ros::Time& time);

    void set_target(state s);

    state get_target() const;

    bool has_reached_target(const arma::rowvec2& agent_pos, const ros::Time& time);

private:

    const arma::mat& grid;

    state target_s;
    double dist_threashold;

    double publish_rate_;
    ros::Time last_publish_time_;
    double dist;




};


}



#endif
