#ifndef OBJECTIVE_FUNCTION_H_
#define OBJECTIVE_FUNCTION_H_

#include "online_search/utils.h"
#include <armadillo>
#include <limits>

namespace search{

inline double objective_function_one(state s_l, state s_c, state s_n,const arma::ucolvec& occupancy, const arma::mat& grid){

    if(s_c == s_n) // did not move
    {
        return -2.0;
    }else if(occupancy(s_n) == 1) // already visited the next state
    {
        return -1.0;
    }else{

        arma::rowvec2 last_motion = grid.row(s_c) - grid.row(s_l);
        arma::rowvec2 next_motion = grid.row(s_n) - grid.row(s_c);
        double dot = arma::dot(last_motion/(arma::norm(last_motion) + std::numeric_limits<double>::min() ),next_motion/(arma::norm(next_motion) + std::numeric_limits<double>::min() ));
        return dot;
    }

}




}




#endif
