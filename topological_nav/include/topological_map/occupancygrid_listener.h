#ifndef OCCUPANCYGRID_LISTENER_H_
#define OCCUPANCYGRID_LISTENER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include "topological_map/topologymap.h"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>

#include <memory>

namespace c2t{

class OccupancyGridListener{

public:

    OccupancyGridListener();

    /**
     * @brief blocks until msg callback is called.
     */
    void wait_msg(ros::NodeHandle &nh) const;

    grid_map::GridMap &get_grid_map();

public:

    void occupancygrid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    double size_x_m, size_y_m;
    double size_y_,size_x_;
    double resolution_;

private:

    ros::Subscriber             sub;
    grid_map::GridMap           grid_map;

    bool                        bFirstMsg;




};

}

#endif
