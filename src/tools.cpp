#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse = VectorXd(4);
    rmse << 0,0,0,0;

    for (int i=0; i < estimations.size();++i){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array() * residual.array();
        rmse += residual;
    }
    rmse = rmse / estimations.size();
    rmse = sqrt(rmse.array());

    return rmse;
  /**
  TODO:
    * Calculate the RMSE here.
  */
}
