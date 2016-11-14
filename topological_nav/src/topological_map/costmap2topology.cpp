#include "topological_map/costmap2topology.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <optitrack_rviz/type_conversion.h>

namespace c2t {
void mat2tf(const arma::mat& m1,tf::Matrix3x3& m2){

    m2[0][0] =  m1(0,0);
    m2[1][0] =  m1(1,0);
    m2[2][0] =  m1(2,0);

    m2[0][1] =  m1(0,1);
    m2[1][1] =  m1(1,1);
    m2[2][1] =  m1(2,1);

    m2[0][2] =  m1(0,2);
    m2[1][2] =  m1(1,2);
    m2[2][2] =  m1(2,2);

}
OccupancyMap2Topology::OccupancyMap2Topology(grid_map::GridMap& map):
map(map){

    //create_topological_map();

    get_points();
    std::cout<< " after get_points" << std::endl;



    std::cout<< "occupancyMap constructore finished" << std::endl;
}

void OccupancyMap2Topology::get_frame_reference(arma::colvec3 &T, arma::mat33 &Rot){

    arma::mat22     Cov;
    arma::colvec2   mean;

    Cov     = arma::cov(b_points);
    mean    = arma::mean(b_points).st();

    T(0) = mean(0);
    T(1) = mean(1);
    T(2) = 0;
    Rot.zeros();
    Rot(0,0) = Cov(0,0);
    Rot(0,1) = Cov(0,1);
    Rot(0,2) = 0;
    Rot(1,0) = Cov(1,0);
    Rot(1,1) = Cov(1,1);
    Rot(1,2) = 0;
    Rot(2,2) = 1;

    arma::vec                       eigval;
    arma::mat                       eigvec;
    arma::eig_sym( eigval, eigvec, Rot );

    Rot.col(0) = -1 * eigvec.col(2);
    Rot.col(1) = -1 * eigvec.col(1);
    Rot.col(2) = eigvec.col(0);


  // tt = T + eigval(2) * Rot.col(0) + eigval(1) * Rot.col(1);



}

void OccupancyMap2Topology::adapt_grid(arma::mat& grid){

    std::size_t idx;
    double threas = 0.5;

    for(int i = 0; i < grid.n_rows;i++)
    {

       if( closest_dist(grid.row(i),b_points,idx) < threas)
       {
        // delete point
           grid.shed_row(i);
       }


    }


}


void OccupancyMap2Topology::get_bbox(){

    arma::mat& points = g_points;


    arma::rowvec2 mean    = arma::mean(points);


    arma::colvec dist(points.n_rows);
    for(std::size_t i = 0; i < points.n_rows;i++)
    {
        dist(i) = arma::norm(mean - points.row(i));
    }

    arma::uvec idx = arma::sort_index(dist);

    tt(0) = points(idx(0),0);
    tt(1) = points(idx(0),1);





}

void OccupancyMap2Topology::get_points(){


    grid_map::Matrix& data = map["layer"];

    int count_g = 0;
    int count_b = 0;

    grid_map::Position position;

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        if(!std::isnan(data(index(0), index(1)))){

            if(data(index(0), index(1)) == 0){
                count_g++;
            }else if(data(index(0), index(1)) == 100){
                count_b++;
            }
        }
    }
    g_points.resize(count_g,2);
    b_points.resize(count_b,2);
    count_g = 0;
    count_b = 0;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        if(!std::isnan(data(index(0), index(1)))){
            map.getPosition(*it, position);
            if(data(index(0), index(1)) == 0){
                g_points(count_g,0) = position.x();
                g_points(count_g,1) = position.y();
                count_g++;
            }else if(data(index(0), index(1)) == 100){
                b_points(count_b,0) = position.x();
                b_points(count_b,1) = position.y();
                count_b++;
            }
        }
    }
}


double OccupancyMap2Topology::closest_dist(const arma::rowvec2& p, const arma::mat& grid, std::size_t &idx) const{

    double closest_dist = arma::norm(p - grid.row(0));
    idx = 0;
    double tmp;
    for(std::size_t i = 1; i < grid.n_rows;i++)
    {

        tmp = arma::norm(p - grid.row(i));
        if(tmp < closest_dist)
        {
            idx = i;
            closest_dist = tmp;
        }

    }

    return closest_dist;

}



void OccupancyMap2Topology::create_topological_map(){




}

const arma::mat& OccupancyMap2Topology::getGrid() const{
    return grid;
}




}
