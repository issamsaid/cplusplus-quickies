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
/// @file fox_matmul/main.cc
/// @author Issam SAID 
/// @brief The main routine that runs the Fox matrix multiplication algorithm.
///
#include <mpi.h>
#include <iostream>
#include <matrix-inl.h>
#include <grid-inl.h>

int  main(int argc, char **argv) {
    unsigned int i, j, n = 2, p = 3, m = 3;
    
    MPI_Init(&argc, &argv);
    //Grid<int> g(1,1);

    Matrix<float> mat1(n, p);
    Matrix<float> mat2(p, m);
    Matrix<float> mat3(n, m);
    
    for (j=0; j < n; j++) 
        for (i=0; i < p; i++) mat1[j][i]= j*p+i;
    for (j=0; j < p; j++) 
        for (i=0; i < m; i++) mat2[j][i]= j*m+i;
        
    //m3 = m1 + m2;
    std::cout << mat1 << std::endl;
    std::cout << mat2 << std::endl;  
    
    mat3 = mat1 * mat2;
    
    std::cout << mat3 << std::endl; 
    
    MPI_Finalize();

    return 0;
}