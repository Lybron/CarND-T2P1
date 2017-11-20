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

  if (estimations.size() == 0 || ground_truth.size() == 0) {
    cout << "RMSE: Estimation or Ground Truth matrix is empty." << endl;
    return rmse;
  }

  if (estimations.size() != ground_truth.size()) {
    cout << "RMSE: Estimation and Ground Truth matrices are different sizes." << endl;
    return rmse;
  }

  // calculate residual
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd res = (estimations[i] - ground_truth[i]);
    res = res.array() * res.array();
    rmse += res;
  }

  // mean
  rmse = rmse/estimations.size();

  // square root
  rmse = rmse.array().sqrt();

  cout << "RMSE:" << endl;
  cout << rmse << endl;

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  // state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //cehck for division by 0
  if (px == 0 || py == 0) {
    cout << "Jacobian: dividing by 0; x or y is zero." << endl;
  }

  float squares = (px*px) + (py*py);
  float sqroot = sqrt(squares);
  float sqr_prod = squares * sqroot;

  MatrixXd j = MatrixXd(3,4);

  cout << "Created matrix" << endl;

  j <<  px/sqroot, py/sqroot,0,0,
        -py/squares,px/squares,0,0,
        py*((vx*py) - (vy*px))/sqr_prod, px*((vy*px) - (vx*py))/sqr_prod, px/sqroot, py/sqroot;

  return j;
}
