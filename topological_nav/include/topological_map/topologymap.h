#ifndef TOPOLOGICALMAP_H_
#define TOPOLOGICALMAP_H_

#include <armadillo>
#include <limits>


namespace topolog_map{


class TopologyMap{


public:

    static double closest_point(const arma::rowvec2& p, const arma::mat& grid, std::size_t &idx);

    /**
    * @brief  Creates a 2D grid stored in a (N * M x 2) matrix
    */
    static void create_2d_grid(arma::mat& grid, const double origin_m,const double origin_n,
                               const double length_m, const double length_n,
                               std::size_t M ,std::size_t N );


    /**
     * @brief Given a set of points [grid] computes the connectivity of the grath.
     */
    static void build_adjacency_matrix(arma::Mat<int>& A, const arma::mat& grid, double max_distance);


    static void partition_space(const arma::mat& grid,std::vector<arma::mat>& grids, std::size_t dim);

};



}

#endif
