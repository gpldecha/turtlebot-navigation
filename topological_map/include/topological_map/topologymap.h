#ifndef TOPOLOGICALMAP_H_
#define TOPOLOGICALMAP_H_

#include <armadillo>

namespace topolog_map{


/**
 * @brief  Creates a 2D grid stored in a (N * M x 2) matrix
 */
inline void create_2d_grid(arma::mat& grid, const double origin_m,const double origin_n, const double length_m, const double length_n,const double resolution){

    std::cout<< "     Creating Grid    " << std::endl;
    std::cout<< "         length_m : " << length_m << std::endl;
    std::cout<< "         length_n : " << length_n << std::endl;
    std::cout<< "       resolution : " << resolution << std::endl;


    std::size_t M       = length_m / resolution;
    std::size_t N       = length_n / resolution;
    std::size_t n_rows  = N * M;

    grid.set_size(n_rows,2);

    // compute coordinate of grid points
    std::size_t r = 0;
    for(std::size_t m = 0; m < M; m++)
    {
        for(std::size_t n = 0; n < N; n++)
        {
            grid(r,0) = (static_cast<double>(m) - origin_m)/resolution;
            grid(r,1) = (static_cast<double>(n) - origin_n)/resolution;
            r++;
        }
    }

    std::cout<< "  GRID: " << n_rows << std::endl;
}

/**
 * @brief Given a set of points [grid] computes the connectivity of the grath.
 */
inline void build_adjacency_matrix(arma::Mat<int>& A, const arma::mat& grid, double max_distance){

    arma::uvec indices;
    arma::vec dist(grid.n_rows);
    A = arma::Mat<int>(grid.n_rows,4,arma::fill::ones);
    A = A * -1;


    //grid.print("grid");

    arma::rowvec tmp(3);
    for(std::size_t i = 0; i < A.n_rows;i++){
        // norm

        for(int j = 0; j < A.n_rows;j++){
            dist(j) = arma::norm(grid.row(i)- grid.row(j)  , 1);
        }

        // find the 5 closest neighbours
        indices = arma::sort_index(dist);

          for(int j=1;j<5;j++){


             tmp = grid.row(indices(j)) - grid.row(i);

            if(tmp(0) == 0 || tmp(1) == 0 && arma::norm(tmp) <= max_distance )
            {
                A(i,j-1) = indices(j);
            }
        }
    }


}




}

#endif
