#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;

/**
 * calculate root mean squared error between kalman filter estimations and ground truth, to evaluate kf performance.
 * @param estimations
 * @param ground_truth
 * @return
 */
VectorXd Tools::CalculateRMSE(const std::vector<VectorXd> &estimations,
                              const std::vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.empty() || estimations.size() != ground_truth.size()) {
    std::cout << "Invalid data\n";
    return rmse;
  }

  // accumulate squared residuals
  for (size_t i=0; i < estimations.size(); i++) {
    // ... your code here
    VectorXd diff = estimations[i] - ground_truth[i];
    rmse = rmse + diff.cwiseProduct(diff);
  }

  // calculate the mean
  rmse /= estimations.size();

  // calculate the squared root
  rmse = rmse.cwiseSqrt();

  // return the result
  return rmse;
}
