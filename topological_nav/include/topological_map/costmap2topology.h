#ifndef COSTMAP2TOPOLOGY_H_
#define COSTMAP2TOPOLOGY_H_

#include <costmap_2d/costmap_2d.h>
#include <memory>
#include <armadillo>
#include "topological_map/topologymap.h"

// should fetch the global cost map once at initialisation
// nav_msgs/OccupancyGrid

#include <grid_map_ros/GridMapRosConverter.hpp>


namespace c2t{


class OccupancyMap2Topology{

public:

    OccupancyMap2Topology(grid_map::GridMap& map);

    /**
     * @returns armadillo martrix with N * M 2D points
     */
    const arma::mat& getGrid() const;

    void get_frame_reference(arma::colvec3 &T, arma::mat33 &Rot);

    void get_bbox();

    void adapt_grid(arma::mat& grid);

private:


    void get_points();

    double closest_dist(const arma::rowvec2& p, const arma::mat&grid, std::size_t& idx )const ;


    void create_topological_map();

public:

    arma::colvec3 tt, bb, bt, tb;
    arma::mat g_points;
    arma::mat b_points;

private:

    grid_map::GridMap& map;
    arma::mat grid;







};


}


#endif
