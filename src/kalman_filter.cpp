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
  this->x_ = this->F_ * this->x_;
  MatrixXd Ft = this->F_.transpose();
  this->P_ = this->F_ * this->P_ * Ft + this->Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = this->H_ * this->x_;
  VectorXd y = z - z_pred;

  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
 
    //Convert vector (rho,  phi and rhoDot)
    float px = this->x_[0];
    float py = this->x_[1];
    float vx = this->x_[2];
    float vy = this->x_[3];

    //first conversion in the vector
    float rho = sqrt(px*px + py*py);
    float phi = atan2(py, px);
    float rho_dot = 0;

    if(fabs(rho) > 0.001){
     rho_dot = (px*vx + py*vy) / rho;
	}

    VectorXd h = VectorXd(3);

    h << rho, phi, rho_dot;

    VectorXd y = z - h;
    
    NormalizeAngle(y(1));

    UpdateCommon(y);
}

void KalmanFilter::NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}

//Because this piece of code is the same for both radar and laser measurements,
//we can re-use the code in a function for both.
// As Suggested by the Udacity reviewer of my previous submission.
void KalmanFilter::UpdateCommon(const VectorXd& y)
{
  const MatrixXd PHt = P_ * H_.transpose();
  const MatrixXd S = H_ * PHt + R_;
  const MatrixXd K = PHt * S.inverse();

  x_ += K * y;
  P_ -= K * H_ * P_;
}