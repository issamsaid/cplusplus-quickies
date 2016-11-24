#ifndef __FOX_MATMUL_MATRIX_INL_H_
#define __FOX_MATMUL_MATRIX_INL_H_
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
/// @file fox_matmul/matrix-inl.h
/// @author Issam SAID
/// @brief A templated class that defines square matrices.
///
#include <iostream>

template<typename T> 
class Matrix {
public:
    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    Matrix(const unsigned int& nbRows, 
           const unsigned int& nbCols) : rows(nbRows), cols(nbCols) {
        data = new T[rows*cols];
        std::cout << "... matrix ctor invoked " << this << std::endl;
    }

    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    Matrix(const Matrix<T>& m) : rows(m.rows), cols(m.cols) {
        data = new T[rows*cols];
        for (int i=0; i<rows*cols; i++) data[i] = m.data[i];
        std::cout << "... matrix copy ctor invoked " << this << std::endl;
    }

    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
   ~Matrix() {
        delete[] data;
        std::cout << "... matrix dtor invoked " << this << std::endl;
    }
    
    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    Matrix& operator=(const Matrix<T>& m) {
        rows = m.rows;
        cols = m.cols;
        for (int i=0; i<rows*cols; i++) data[i] = m.data[i];
        std::cout << "... matrix assign op invoked " << this << std::endl;
        return *this;
    }

    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    const T * operator[](const unsigned int& j) const {
        return &data[j*cols];
    }

    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    T * const operator[](const unsigned int& j) {
        return &data[j*cols];
    }

    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    Matrix<T> operator+(const Matrix<T>& m) {
        Matrix<T> mRes(rows, cols);
        for (unsigned int j=0; j < rows; j++) {
            for (unsigned int i=0; i < cols; i++) {
                mRes.data[j*cols+i] = this->data[j*cols+i] + m.data[j*cols+i];
            }
        }
        return mRes;
    }

    ///
    /// @brief Overload the multiplication operator for the Matrix class
    /// @param m is the second matrix to be multiplied by the current matrix  
    ///        NxP * PxM = NxM
    /// @return The resulting Matrix 
    ///
    Matrix<T> operator*(const Matrix<T>& m) {
        //if (this->cols != m.rows) throw();
        Matrix<T> mRes(this->rows, m.cols);
        for (unsigned int j=0; j < this->rows; j++) {
            for (unsigned int i=0; i < m.cols; i++) {
                for (unsigned int k=0; k < m.rows; k++) {
                    mRes.data[j*m.cols+i] += 
                        this->data[j*this->cols+k]*m.data[k*m.cols+i];
                }
            }
        }
        return mRes;
    }

    ///
    /// @brief
    ///
    ///
    /// @param
    /// @return
    ///
    friend std::ostream& operator<<(std::ostream& os, Matrix<T> const& m) {
        for (unsigned int j=0; j < m.rows; j++) {
            for (unsigned int i=0; i < m.cols; i++) {
                os << m.data[j*m.cols+i] << " "; 
            }
            os << std::endl;
        } 
        return os; 
    }

private:
    unsigned int rows;
    unsigned int cols;
    T *data;
};

// template<class T>
// auto operator<<(std::ostream& os, const T& t) -> decltype(t.print(os), os) 
// { 
//     t.print(os); 
//     return os; 
// } 

#endif // __FOX_MATMUL_MATRIX_INL_H_
