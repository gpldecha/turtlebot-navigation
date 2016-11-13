#include "topological_map/occupancygrid_listener.h"

namespace c2t{

OccupancyGridListener::OccupancyGridListener(const std::string &topic_name, ros::NodeHandle& nh):
nh(nh){

    nh.subscribe(topic_name, 1, &OccupancyGridListener::occupancygrid_callback, this);

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

void OccupancyGridListener::wait_msg() const{
    ROS_WARN("Waiting for nav_msgs::OccupancyGrid message.");
    ros::Rate rate(10);
    while(nh.ok() && !bFirstMsg){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("nav_msgs::OccupancyGrid message RECEIVED!");
}

const costmap_2d::Costmap2D &OccupancyGridListener::getCostMap() const{
    return costmap_2D;
}


// costmap_2d_publisher.cpp published message: nav_msgs::OccupancyGrid
void OccupancyGridListener::occupancygrid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){


    /*
     *
        double resolution = costmap_->getResolution();

        grid_.header.frame_id = global_frame_;
        grid_.header.stamp = ros::Time::now();
        grid_.info.resolution = resolution;

        grid_.info.width = costmap_->getSizeInCellsX();
        grid_.info.height = costmap_->getSizeInCellsY();

        double wx, wy;
        costmap_->mapToWorld(0, 0, wx, wy);
        grid_.info.origin.position.x = wx - resolution / 2;
        grid_.info.origin.position.y = wy - resolution / 2;
        grid_.info.origin.position.z = 0.0;
        grid_.info.origin.orientation.w = 1.0;
        saved_origin_x_ = costmap_->getOriginX();
        saved_origin_y_ = costmap_->getOriginY();

  */

    /*unsigned int Costmap2D::getSizeInCellsY() const
{
  return size_y_;
}

double Costmap2D::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}*/


    msg->info.resolution;

    msg->info.height; //
    msg->info.width;

    // size in cell
    msg->info.height;

    // size in meters
    //(size_x_ - 1 + 0.5) * resolution_

    costmap_2D = costmap_2d::Costmap2D(msg->info.width,msg->info.height,msg->info.resolution,msg->info.origin.position.x, msg->info.origin.position.y);


  /*  unsigned int i = 0;
     for (unsigned int y = y0_; y < yn_; y++)
     {
       for (unsigned int x = x0_; x < xn_; x++)
       {
           costmap_2D.setCost();

         //unsigned char cost = costmap_->getCost(x, y);
         //update.data[i++] = cost_translation_table_[ cost ];
       }
     }*/


/*
    costmap_2D =


            */
    bFirstMsg = true;


}


}
