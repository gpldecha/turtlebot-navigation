#ifndef OCCUPANCYGRID_LISTENER_H_
#define OCCUPANCYGRID_LISTENER_H_

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>

namespace c2t{

class OccupancyGridListener{

public:

    OccupancyGridListener(const std::string& topic_name,ros::NodeHandle& nh);

    /**
     * @brief blocks until msg callback is called.
     */
    void wait_msg() const;

    const costmap_2d::Costmap2D& getCostMap() const;

private:

    void occupancygrid_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

private:

    ros::NodeHandle&            nh;
    ros::Subscriber             sub;
    costmap_2d::Costmap2D       costmap_2D;
    char*                       cost_translation_table_;
    bool                        bFirstMsg;


};

}

#endif
