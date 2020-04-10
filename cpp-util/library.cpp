#include <iostream>
#include "Eigen/Dense"
#include "drake/math/discrete_algebraic_riccati_equation.h"

using Eigen::MatrixXd;

extern "C" __declspec(dllexport)
void discreteAlgebraicRiccatiEquation(double *A, double *B, double *Q, double *R, int states, int inputs, double** ppVals, int* pNumVals)
{
    Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            Amat{A, states, states};
    Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            Bmat{B, states, inputs};
    Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            Qmat{Q, states, states};
    Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            Rmat{R, inputs, inputs};


    Eigen::MatrixXd result = drake::math::DiscreteAlgebraicRiccatiEquation(Amat, Bmat, Qmat, Rmat);

    int loop = 0;
    *pNumVals = result.size();
    *ppVals = (double*)malloc(sizeof(double) * *pNumVals);
    memset(*ppVals, 0, sizeof(double) * *pNumVals);
    for (loop=0; loop<*pNumVals; loop++)
    {
        // populate the array with junk data (just for the sake of the example)
        (*ppVals)[loop] = result.data()[loop];
    }
}


extern "C" __declspec(dllexport)
void cleanup(double* pVals)
{
    free(pVals);
}