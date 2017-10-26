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

  VectorXd rmse(4), diff(4);
  int size = estimations.size();
  rmse << 0.0, 0.0, 0.0, 0.0;
 
  if(size != ground_truth.size() || size == 0) {
    cout << "size error to calculate RMSE!";
    return rmse;
  }

  for(int i = 0; i < size; i++) {
    diff = (estimations[i] - ground_truth[i]);
    rmse += (VectorXd)(diff.array() * diff.array());    
  }

  rmse /= size;
  rmse = rmse.array().sqrt();
  cout << "rmse:" << size << "\n" << rmse << "\n";
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);


  float var1, var2, var3;
  var1 = px * px + py * py;
  var2 = vx * py - vy * px;
  var3 = sqrt(var1);

  if (var1 < 0.0001)
  {
    cout << "divide by 0 when calculating Jacobian\n";
    return Hj;
  }
  Hj << px / var3, py / var3, 0, 0,
        - py / var1, px / var1, 0, 0,
        py * var2 / (var1 * var3), -px * var2 / (var1 * var3), px / var3, py / var3;

  return Hj;
}
