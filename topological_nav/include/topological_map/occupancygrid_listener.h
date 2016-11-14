#ifndef OCCUPANCYGRID_LISTENER_H_
#define OCCUPANCYGRID_LISTENER_H_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include "topological_map/topologymap.h"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <memory>

namespace c2t{

class OccupancyGridListener{

public:

    OccupancyGridListener();

    /**
     * @brief blocks until msg callback is called.
     */
    void wait_msg(ros::NodeHandle &nh) const;

  //  const costmap_2d::Costmap2D& getCostMap() const;

    const topolog_map::Map& get_map() const;

    grid_map::GridMap &get_grid_map();

public:

    void occupancygrid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    double size_x_m, size_y_m;
    double size_y_,size_x_;
    double resolution_;

private:

    ros::Subscriber             sub;
    costmap_2d::Costmap2D       costmap_2D;
    topolog_map::Map            map;
    grid_map::GridMap           grid_map;

    char*                       cost_translation_table_;
    bool                        bFirstMsg;




};

}

#endif
