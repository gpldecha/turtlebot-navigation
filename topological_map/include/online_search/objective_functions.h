#ifndef OBJECTIVE_FUNCTION_H_
#define OBJECTIVE_FUNCTION_H_

#include "online_search/utils.h"
#include <armadillo>

namespace search{

//arma::ucolvec& occupancy, arma::mat& grid
inline double objective_function_one(state s_l, state s_c, state s_n,const arma::ucolvec& occupancy, const arma::mat& grid){

    if(s_c == s_n) // did not move
    {
        return -1;
    }else if(occupancy(s_n) == 1) // already visited the next state
    {
        return -0.5;
    }else{

        arma::rowvec2 diff = grid.row(s_n) - grid.row(s_l);
        if(diff(0) == 0 || diff(1) == 0) // travelled in a straight line
        {
            return 1;
        }else{
            return 0;
        }
    }

}




}




#endif
