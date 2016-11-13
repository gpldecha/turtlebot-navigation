#ifndef COSTMAP2TOPOLOGY_H_
#define COSTMAP2TOPOLOGY_H_

#include <costmap_2d/costmap_2d.h>
#include <memory>
#include <armadillo>
#include "topological_map/topologymap.h"

// should fetch the global cost map once at initialisation
// nav_msgs/OccupancyGrid


namespace c2t{


class CostMap2Topology{

public:

    CostMap2Topology(const costmap_2d::Costmap2D& costmap);

    /**
     * @returns armadillo martrix with N * M 2D points
     */
    const arma::mat& getGrid() const;

private:

    void create_topological_map();

private:

    const costmap_2d::Costmap2D& costmap;
    arma::mat grid;

};


}


#endif
