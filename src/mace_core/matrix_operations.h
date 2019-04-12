#ifndef MATRIXOPERATIONS_H
#define MATRIXOPERATIONS_H

#include "Eigen/Core"

#include <vector>

//!
//! \brief Structure defining a modication to a single cell of a matrix
//!
template <typename T>
struct MatrixCellData
{
    int i;
    int j;
    T data;
};


//!
//! \brief Replace a set of cells in a matrix
//! \param mat matrix to operate on
//! \param operations Cells to replace with
//!
static void ReplaceCellsInMatrix(Eigen::MatrixXd &mat, const std::vector<MatrixCellData<double>> &operations)
{
    for(MatrixCellData<double> cell : operations)
    {
        mat(cell.i, cell.j) = cell.data;
    }
}


//!
//! \brief Replace a set of cells in a matrix
//! \param mat matrix to operate on
//! \param operations Cells to replace with
//!
static void ReadCellsInMatrix(Eigen::MatrixXd &mat, std::vector<MatrixCellData<double>> &operations)
{
    for(MatrixCellData<double> cell : operations)
    {
        cell.data = mat(cell.i, cell.j);
    }
}

#endif // MATRIXOPERATIONS_H
