#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  //initialize the matrix to return as a result of calculating the Jacobian
  MatrixXd Hj(3,4);

  //fetch px py vx vy from the state received as parameter
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //assign variables to optimize computation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  //validate that jacobian does not divide by 0
  if(fabs(c1) < 0.0001){
      return Hj;
  }

  //Compute the actual Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
