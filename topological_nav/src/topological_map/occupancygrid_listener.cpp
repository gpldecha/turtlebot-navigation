#include "topological_map/occupancygrid_listener.h"

namespace c2t{

OccupancyGridListener::OccupancyGridListener(){

   // nh.subscribe(topic_name, 100, &OccupancyGridListener::occupancygrid_callback, this);
    //costmap_2D_ros_ptr.reset( new costmap_2d::Costmap2DROS()) ;



    bFirstMsg = false;

    cost_translation_table_ = NULL;

    if (cost_translation_table_ == NULL)
    {
      cost_translation_table_ = new char[256];

      // special values:
      cost_translation_table_[0] = 0;  // NO obstacle
      cost_translation_table_[253] = 99;  // INSCRIBED obstacle
      cost_translation_table_[254] = 100;  // LETHAL obstacle
      cost_translation_table_[255] = -1;  // UNKNOWN

      // regular cost values scale the range 1 to 252 (inclusive) to fit
      // into 1 to 98 (inclusive).
      for (int i = 1; i < 253; i++)
      {
        cost_translation_table_[ i ] = char(1 + (97 * (i - 1)) / 251);
      }
    }

}
/*
void OccupancyGridListener::get_map(ros::NodeHandle& nh){
    nh.subscribe("/map", 100, &OccupancyGridListener::occupancygrid_callback, this);
}
*/
void OccupancyGridListener::wait_msg(ros::NodeHandle& nh) const{
    ROS_WARN("Waiting for nav_msgs::OccupancyGrid message.");
    ros::Rate rate(50);
    while(nh.ok() && !bFirstMsg){


        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("nav_msgs::OccupancyGrid message RECEIVED!");
}

/*
const costmap_2d::Costmap2D &OccupancyGridListener::getCostMap() const{
    return costmap_2D;
}
*/

const topolog_map::Map& OccupancyGridListener::get_map() const{
    return map;
}

grid_map::GridMap& OccupancyGridListener::get_grid_map() {
    return grid_map;
}

// costmap_2d_publisher.cpp published message: nav_msgs::OccupancyGrid
void OccupancyGridListener::occupancygrid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std::cout<< "occupancygrid_callback !! " << std::endl;


    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg,"layer",grid_map);

    const std::vector<std::string>& layers = grid_map.getLayers();
    for(std::size_t i = 0; i < layers.size();i++)
    {
        std::cout<< "layers("<<i<<"): " << layers[i] << std::endl;
    }


    size_y_     = msg->info.height;           //  costmap_->getSizeInCellsY() = size_y_;
    size_x_     = msg->info.width;            //  costmap_->getSizeInCellsX() = size_x_;
    resolution_ = msg->info.resolution;       //  resolution = costmap_->getResolution();

    msg->info.origin.position;  /* grid_.info.origin.position.x     = wx - resolution / 2;
                                   grid_.info.origin.position.y     = wy - resolution / 2;
                                   grid_.info.origin.position.z     = 0.0;
                                   grid_.info.origin.orientation.w  = 1.0;
                                 */
    map.origin(0) = msg->info.origin.position.x;
    map.origin(1) = msg->info.origin.position.y;
    map.origin(2) = msg->info.origin.position.z;

       /*double Costmap2D::getSizeInMetersX() const
       return (size_x_ - 1 + 0.5) * resolution_;
    */

    size_x_m = (size_x_ - 1 + 0.5)  * resolution_;
    size_y_m = (size_y_ - 1 + 0.5)  * resolution_;

    std::cout<< "Info " << std::endl;
    std::cout<< "size_x_m:      " << size_x_m    << std::endl;
    std::cout<< "size_y_m:      " << size_y_m       << std::endl;
    std::cout<< "size_y_:       " << size_y_        << std::endl;
    std::cout<< "size_x_:       " << size_x_        << std::endl;
    std::cout<< "resolution_:   " << resolution_    << std::endl;


    bFirstMsg = true;

}


}
