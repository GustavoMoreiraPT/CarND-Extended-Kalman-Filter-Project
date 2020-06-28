#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  this->x_ = this->F_ _ this->x_;
  MatrixXd Ft = this->F_.transpose();
  this->P = this->F_ * this->P_ * Ft + this->Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = this->H_ * this->x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = this->H_.transpose();
  MatrixXd S = this->H_ * this->P_ * Ht * this->R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = this->P * Ht;
  MatrixXd K = PHt * Si;

  //here we do the new estimation by updtating the values of matrix P and vector x_
  this-> x = this->x + (K * y);
  long x_size = this.x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  this->P_ = (I - k * this->H_) * this->P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
