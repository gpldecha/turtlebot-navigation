#ifndef ONLINE_SEARCH_UTILS_H_
#define ONLINE_SEARCH_UTILS_H_

#include <functional>
#include <array>
#include <armadillo>
#include <ros/ros.h>
#include "visualise/vis_utils.h"
#include <tf/LinearMath/Vector3.h>

namespace search{

typedef int state;

/**
 * @brief objective_func (same as reward fucntion), given a current
 * state, action and resulting next state returns the utility of this
 * transition.
 */
typedef std::function<double(state,state,state)> objective_func;

enum class action {one=0, two=1, three=2, four=3};

inline std::size_t action2int(action u)
{
    switch(u){
    case action::one:
    {
        return 0;
    }
    case action::two:
    {
        return 1;
    }
    case action::three:
    {
        return 2;
    }
    case action::four:
    {
        return 3;
    }
    }
}


inline state get_state(const arma::rowvec2& position, const arma::mat& grid){
    double dist = arma::norm(position - grid.row(0));
    double tmp  = 0;
    state s = 0;
    for(std::size_t j = 1; j < grid.n_rows;j++){
        tmp = arma::norm(position - grid.row(j));
        if(tmp < dist)
        {
            dist = tmp;
            s  = j;
        }
    }
    return s;
}

/**
  * @brief forward_dynamics   : computes the next state given current state and auction.
  * @param state              : current state.
  * @param a                  : applied action.
  * @param Adj                : adjacency matrix representing connectivity between
  *                             neighbouring states.
  * @return                   : next state.
*/
inline std::size_t forward_dynamics(state s, action a, const arma::Mat<int>& Adj){
    if(s < Adj.n_rows)
    {
        if(Adj(s,action2int(a)) == -1) // boundary stay in the same position
        {
            return s;
        }else {
            return Adj(s,action2int(a));
        }
    }else{
        std::cerr<< "[utils::forward_dynamics] state(" << s <<") > Adj.n_rows(" << Adj.n_rows << ")" << std::endl;
    }
}

/**
 * @brief get_velocity      : simple linear attractor.
 * @param velocity          :
 * @param position          : current Cartesian position
 * @param target            : target Cartesian position
 * @param max_speed         : maximum speed
 * @param period            : loop cycle period dt
 */
inline void get_velocity(arma::colvec3& velocity, const arma::colvec3& position,const arma::colvec3& target, double max_speed, const double period)
{

    velocity = (target - position) * period;
    if(arma::norm(velocity) > max_speed)
    {
        velocity = velocity / arma::norm(velocity) * max_speed;
    }
}

inline void set_color(const arma::ucolvec& occupancy,opti_rviz::acolor& colors){
    if(colors.size() != occupancy.n_elem)
    {
        colors.resize(occupancy.n_elem);
    }
    for(std::size_t i = 0; i < occupancy.n_elem;i++)
    {
        if(occupancy(i) == 0) // not visited
        {
            opti_rviz::set_color(colors[i],1,opti_rviz::COLORS::RED);
        }else{ // visited
            opti_rviz::set_color(colors[i],1,opti_rviz::COLORS::GREEN);
        }
    }
}

inline void get_neighbours(std::vector<tf::Vector3>& neighbours,state s, const arma::Mat<int>& Adj,const arma::mat& grid){

    neighbours.clear();
    assert(s >= 0 && s < Adj.n_rows);

    int idx = 0;
    for(int i = 0; i < 4; i++){
        idx = Adj(s,i);
        if(idx != -1){
            neighbours.push_back(tf::Vector3( grid(idx,0), grid(idx,1), 0  ));
        }
    }
}


}


#endif
