#ifndef ONLINE_PLANNER_H_
#define ONLINE_PLANNER_H_

#include <armadillo>
#include <functional>
#include "online_search/utils.h"

namespace search{

class Online_search{

public:

    Online_search(const arma::Mat<int>& Adj, const objective_func &obj_func);

    /**
     * @brief finds an action which optimises the objective function
     *        for a T=2 planning horizon.
     **/
    action get_action(state s_c, state s_l, std::size_t max_depth=2);

    double get_value(action a);

    void print() const;

private:

    double get_value(state s);

    void depth_first_search(state s_l, state s_c, double &value, action &optimal_action, std::size_t depth, std::size_t max_depth);

    void get_max(double& value,action& a, const std::array<double,4>& Q);

private:

    const arma::Mat<int>&       Adj;
    const objective_func&       obj_func;

    std::array<action,4>        actions = {{action::one,action::two,action::three,action::four}};
    std::array<double,4>        Q       = {{-10,-10,-10,-10}};

    action                      optimal_action;

    double                      dist,tmp,value;

};

}


#endif
