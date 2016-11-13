#ifndef ONLINE_PLANNER_H_
#define ONLINE_PLANNER_H_

#include <armadillo>
#include <functional>
#include "online_search/utils.h"

namespace search{

class Online_search{

public:

    Online_search(const arma::Mat<int>& Adj, const objective_func &obj_func);

    void update_state(const arma::rowvec2& agent_pos, const arma::mat &grid);

    /**
     * @brief finds an action which optimises the objective function
     *        for a T=2 planning horizon.
     */
    action get_action(std::size_t max_depth=2);

    void print() const;

private:

    double get_value(state s);

    void depth_first_search(state s_l, state s_c, double &value, action &optimal_action, std::size_t depth, std::size_t max_depth);

    void get_max(double& value,action& a, const std::array<double,4>& Q);

private:

    const arma::Mat<int>&       Adj;
    const objective_func&       obj_func;

    std::array<action,4>        actions = {{action::one,action::two,action::three,action::four}};
    std::array<double,4>        Q       = {{0,0,0,0}};
    std::array<double,4>        Qtmp    = {{0,0,0,0}};

    state                       s_l, s_c;

    action                      optimal_action;

    double                      dist,tmp,value;

};

}


#endif
