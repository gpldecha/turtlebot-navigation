#include "topological_map/occupancygrid_listener.h"

namespace c2t{

OccupancyGridListener::OccupancyGridListener(){
    bFirstMsg = false;
}

void OccupancyGridListener::wait_msg(ros::NodeHandle& nh) const{
    ROS_WARN("Waiting for nav_msgs::OccupancyGrid message.");
    ros::Rate rate(50);
    while(nh.ok() && !bFirstMsg){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("nav_msgs::OccupancyGrid message RECEIVED!");
}

grid_map::GridMap& OccupancyGridListener::get_grid_map() {
    return grid_map;
}

void OccupancyGridListener::occupancygrid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std::cout<< "occupancygrid_callback !! " << std::endl;
    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg,"layer",grid_map);
    bFirstMsg = true;
/*
    size_y_     = msg->info.height;           //  costmap_->getSizeInCellsY() = size_y_;
    size_x_     = msg->info.width;            //  costmap_->getSizeInCellsX() = size_x_;
    resolution_ = msg->info.resolution;       //  resolution = costmap_->getResolution();
*
    msg->info.origin.position;  /* grid_.info.origin.position.x     = wx - resolution / 2;
                                   grid_.info.origin.position.y     = wy - resolution / 2;
                                   grid_.info.origin.position.z     = 0.0;
                                   grid_.info.origin.orientation.w  = 1.0;
                                 */
   /*
    size_x_m = (size_x_ - 1 + 0.5)  * resolution_;
    size_y_m = (size_y_ - 1 + 0.5)  * resolution_;
    */


}


}
