#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size() == 0 || estimations.size() != ground_truth.size())
  {
      return rmse;
  }

  //accumulating squared residuals
  for(int i=0; i < estimations.size(); ++i)
  {
      VectorXd temp = estimations[i] - ground_truth[i];
      temp = temp.array()*temp.array();
      rmse += temp;
  }

  //calculating the mean
  rmse = rmse/estimations.size();

  //calculating rmse
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

  // Fetching state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //compute the Jacobian matrix
  double k = pow(px,2) + pow(py, 2);
  double l = vx*py - vy*px;
      
  Hj << px/pow(k, 0.5), py/pow(k, 0.5), 0, 0,
        -py/k, px/k, 0, 0,
        py*l/pow(k,1.5), -px*l/pow(k, 1.5), px/pow(k, 0.5), py/pow(k, 0.5);
  

  return Hj;
}
