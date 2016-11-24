#ifndef __FOX_MATMUL_GRID_INL_
#define __FOX_MATMUL_GRID_INL_
///
/// @copyright Copyright (c) 2016-, Issam SAID <said.issam@gmail.com>
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions
/// are met:
///
/// 1. Redistributions of source code must retain the above copyright
///    notice, this list of conditions and the following disclaimer.
/// 2. Redistributions in binary form must reproduce the above copyright
///    notice, this list of conditions and the following disclaimer in the
///    documentation and/or other materials provided with the distribution.
/// 3. Neither the name of the copyright holder nor the names of its
///    contributors may be used to endorse or promote products derived from
///    this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
/// INCLUDING, BUT NOT LIMITED TO, WARRANTIES OF MERCHANTABILITY AND FITNESS
/// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
/// HOLDER OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
/// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
/// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
/// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
/// LIABILITY, WETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
/// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
/// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
/// @file fox_matmul/grid-inl.h
/// @author Issam SAID
/// @brief
///
#include <mpi.h>
#include <math.h>

///
/// @brief Define a 2d periodic Cartesian grid. 
///
template<class T>
class Grid {
public:
    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    Grid(const int& nbRows, const int& nbCols) {
        int p;
        int tmp[maxDims] = {1, 1};
        dim[0] = nbCols;
        dim[1] = nbRows; 
        MPI_Comm_size(MPI_COMM_WORLD, &p);
        // H*W == p
        MPI_Cart_create(MPI_COMM_WORLD, 2, dim, tmp, 1, &comm);
        MPI_Comm_rank(comm, &rank);
        MPI_Cart_coords(comm, rank, maxDims, coords);
        tmp[0] = 1;
        tmp[1] = 0;
        MPI_Cart_sub(comm, tmp, &row_comm);
        tmp[0] = 0;
        tmp[1] = 1;
        MPI_Cart_sub(comm, tmp, &col_comm);
    }

    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
   ~Grid() {
    }

    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    Grid(const Grid&) = delete;
    
    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    Grid& operator=(const Grid&) = delete;

private:
    const static int maxDims = 2;
    int             dim[maxDims];
    int          coords[maxDims];
    int          rank; //< the MPI rank of the current process.
    MPI_Comm     comm; //< the communicator for all the processes.
    MPI_Comm row_comm; //< the communicator for the current process row.
    MPI_Comm col_comm; //< the communicator for the current process column.
    
};

#endif // __FOX_MATMUL_GRID_INL_