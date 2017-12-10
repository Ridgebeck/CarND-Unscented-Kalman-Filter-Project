#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
  /**
  TODO:
    * Calculate the RMSE here.
  */

  // initialize RMSE vector
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // estimation vector size should not be zero
  if(estimations.size() == 0)
  {
    cout << "Estimation vector is empty." << endl;
    return rmse;
  }

  // estimation vector size must be equal ground truth vector size
  if(estimations.size() != ground_truth.size())
  {
    cout << "Invalid estimation or ground_truth data. Data must have the same size." << endl;
    return rmse;
  }

  // accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate rmse value
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;  
}