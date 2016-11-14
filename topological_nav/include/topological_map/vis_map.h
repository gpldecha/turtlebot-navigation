#ifndef TOPOLOGY_MAP_VIS_MAP_H_
#define TOPOLOGY_MAP_VIS_MAP_H_

#include "topologymap.h"
#include "visualise/vis_points.h"
#include "visualise/vis_gmm.h"

#include <tf/LinearMath/Vector3.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace topolog_map{


class Vis_map{

public:

    Vis_map(grid_map::GridMap &map,
            const arma::mat& g_points,
            const arma::mat& b_points,
            std::string& topic_name,
            std::string& fixed_frame,
            ros::NodeHandle &nh);

    void publish();

private:

    std::string fixed_frame;

    grid_map::GridMap &map;
    const arma::mat &g_points;
    const arma::mat &b_points;


    std::unique_ptr<opti_rviz::Vis_points> vis_points_ptr;
    std::vector<tf::Vector3> points;
};



}


#endif
