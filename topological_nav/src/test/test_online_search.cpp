#include <iostream>
#include <online_search/online_search.h>
#include "topological_map/rviz_topology.h"
#include "topological_map/occupancygrid_listener.h"
#include "topological_map/costmap2topology.h"
#include "online_search/online_search.h"

#include "online_search/objective_functions.h"

#include <memory>
#include <functional>

int main(int argc, char** argv)
{

    std::cout<< "testing online search" << std::endl;


    arma::mat grid;
    topolog_map::create_2d_grid(grid,0,0,10,10,10,10);

    arma::Mat<int> Adj;
    topolog_map::build_adjacency_matrix(Adj,grid,1);

    arma::ucolvec occupancy(Adj.n_rows,arma::fill::zeros);


    search::objective_func obj_func =  std::bind(search::objective_function_one,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3,
                                                 std::ref(occupancy),
                                                 std::ref(grid));


    search::Online_search online_search(Adj,obj_func);

    std::cout<< " One line search initialised " << std::endl;
    arma::rowvec2 agent_pos;

    return 0;
}
