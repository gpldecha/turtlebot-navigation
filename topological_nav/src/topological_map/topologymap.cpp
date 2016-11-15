#include "topological_map/topologymap.h"

namespace topolog_map{


double TopologyMap::closest_point(const arma::rowvec2& p, const arma::mat& grid, std::size_t &idx){
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


/**
 * @brief  Creates a 2D grid stored in a (N * M x 2) matrix
 */
void TopologyMap::create_2d_grid(arma::mat& grid, const double origin_m,const double origin_n,
                           const double length_m, const double length_n,
                           std::size_t M ,std::size_t N ){

    std::cout<< "     Creating Grid    " << std::endl;

    std::size_t n_rows  = N * M;

    double resolution_m = length_m / static_cast<double>(M);
    double resolution_n = length_n / static_cast<double>(N);

    double tran_m = -length_m / 2.0 + origin_m;
    double tran_n = -length_n / 2.0 + origin_n;

    grid.set_size(n_rows,2);

    // compute coordinate of grid points
    std::size_t r = 0;
    for(std::size_t m = 0; m < M; m++)
    {
        for(std::size_t n = 0; n < N; n++)
        {
            grid(r,0) =  static_cast<double>(m) * resolution_m + tran_m;
            grid(r,1) =  static_cast<double>(n) * resolution_n + tran_n;
            r++;
        }
    }

    std::cout<< "  GRID: " << n_rows << std::endl;
}



/**
 * @brief Given a set of points [grid] computes the connectivity of the grath.
 */
void TopologyMap::build_adjacency_matrix(arma::Mat<int>& A, const arma::mat& grid, double max_distance){

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

            //tmp(0) == 0 || tmp(1) == 0 &&
           // if(arma::norm(tmp,1) <= max_distance )
           // {
            A(i,j-1) = indices(j);
            // }
        }
    }


}


void TopologyMap::partition_space(const arma::mat& grid,std::vector<arma::mat>& grids, std::size_t dim){

    // first partition along dimension 0 or 1
    // find boundary
    double min, max;
    std::size_t min_i, max_i;

    min = std::numeric_limits<double>::max();
    max = std::numeric_limits<double>::min();

    //    std::cout<< "grid: " << grid.n_rows << " x " << grid.n_cols << std::endl;

    for(std::size_t i = 0; i < grid.n_rows;i++)
    {
        if(grid(i,dim) < min){
            min     = grid(i,dim);
            min_i   = i;
        }
        if(grid(i,dim) > max){
            max     = grid(i,dim);
            max_i   = i;
        }
    }



    double decision_step = (max + min) / 2.0;

   std::cout<< "max: " << max << " min: " << min << " decision: " << decision_step << std::endl;

    /*
     *
     * See Armadillo's documentation. For example, you can use X.n_elem to get the length of vector X.
     * To resize a vector while preserving the data, use .resize(). To add a row or column to a matrix,
     * use .insert_rows() or .insert_cols().*/

    // partition gird into two
    grids.resize(2);
    std::size_t size;
    int count_one, count_two;
    count_one = 0;
    count_two = 0;
    for(std::size_t i = 0; i < grid.n_rows;i++)
    {
        if(grid(i,dim) < decision_step)
        {
            count_one++;
        }else{
            count_two++;
        }
    }

    grids[0].set_size(count_one,2);
    grids[1].set_size(count_two,2);

    count_one = 0;
    count_two = 0;
    for(std::size_t i = 0; i < grid.n_rows;i++)
    {

        if(grid(i,dim) < decision_step)
        {
            grids[0].row(count_one) = grid.row(i);
            count_one++;
        }else{
            grids[1].row(count_two) = grid.row(i);
            count_two++;
        }
    }



}

}
