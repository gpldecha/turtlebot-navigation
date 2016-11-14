#include "topological_map/vis_map.h"
#include <optitrack_rviz/debug.h>

namespace topolog_map {


Vis_map::Vis_map(grid_map::GridMap &map,
                 const arma::mat &g_points,
                 const arma::mat &b_points,
                 std::string& topic_name,
                 std::string& fixed_frame,
                 ros::NodeHandle &nh):
    map(map),
    g_points(g_points),
    b_points(b_points),
    fixed_frame(fixed_frame)
{

    const arma::mat& p = b_points;

    // publish only points value = 100
    points.resize(p.n_rows);

    for (std::size_t i = 0; i < points.size();i++) {
        points[i].setX(p(i,0));
        points[i].setY(p(i,1));
        points[i].setZ(0);
    }

    vis_points_ptr.reset( new opti_rviz::Vis_points(nh,topic_name) );

    vis_points_ptr->r = 1; vis_points_ptr->g = 0; vis_points_ptr->b = 1;
    vis_points_ptr->scale = 0.05;
    vis_points_ptr->initialise(fixed_frame,points);

}



void Vis_map::publish(){

    vis_points_ptr->publish();


}


}
