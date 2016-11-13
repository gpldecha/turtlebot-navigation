#include "finite_state_machine/finite_state_machine.h"

namespace search{

FSM::FSM(const arma::mat& grid, double dist_threashold, double publish_rate_, const ros::Time &time):
    grid(grid),
    dist_threashold(dist_threashold),
    publish_rate_(publish_rate_)
{
    last_publish_time_ = time;
}

void FSM::set_target(state s){
    target_s = s;
}

state FSM::get_target() const{
    return target_s;
}

bool FSM::has_reached_target(const arma::rowvec2 &agent_pos, const ros::Time& time){

    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(publish_rate_) < time) {
        ROS_INFO_STREAM("Check if reached target state");

        last_publish_time_ = last_publish_time_ + ros::Duration(publish_rate_);
        dist = arma::norm(agent_pos - grid.row(target_s));

        if(dist < dist_threashold)
        {
            return true;
        }else{
            return false;
        }
    }

    return false;
}



}
