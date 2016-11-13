#include "topological_map/costmap2topology.h"

namespace c2t {


CostMap2Topology::CostMap2Topology(const costmap_2d::Costmap2D& costmap):
costmap(costmap){

    create_topological_map();

}


void CostMap2Topology::create_topological_map(){

    double origin_x = costmap.getOriginX();
    double origin_y = costmap.getOriginY();

    double size_x    = costmap.getSizeInMetersX();
    double size_y    = costmap.getSizeInMetersY();

    topolog_map::create_2d_grid(grid,origin_x,origin_y,size_x,size_y,1);


}

const arma::mat& CostMap2Topology::getGrid() const{
    return grid;
}




}
