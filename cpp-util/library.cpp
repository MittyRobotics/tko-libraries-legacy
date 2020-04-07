#include <iostream>
#include "Eigen/Dense"
#include "drake/discrete_algebraic_riccati_equation.h"

using Eigen::MatrixXd;

extern "C" __declspec(dllexport)
double discreteAlgebraicRiccatiEquation(double *A, double *B, double *Q, double *R, int states, int inputs,
                                        double **output)
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


    Eigen::MatrixXd result = math::DiscreteAlgebraicRiccatiEquation(Amat, Bmat, Qmat, Rmat);

    std::cout << result.data()[0];

    int loop = 0;
    double pNumVals = states*states;
    *output = (double*)malloc(sizeof(double) * pNumVals);
    memset(*output, 0, sizeof(double) * pNumVals);
    for (loop=0; loop<pNumVals; loop++)
    {
        // populate the array with junk data (just for the sake of the example)
        (*output)[loop] = result.data()[loop];
    }

    return *output[0];
}