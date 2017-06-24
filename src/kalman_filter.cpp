#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  KFHelper(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  VectorXd hx = VectorXd(3);
  float px, py, vx, vy;
  px = x_(0);
  py = x_(1);
  vx = x_(2);
  vy = x_(3);
  float rho = sqrt(pow(px, 2) + pow(py, 2));
  float phi = atan2(py, px);
  float rho_dot = (px*vx+py*vy)/rho;
  hx << rho, phi, rho_dot;
  VectorXd y = z - hx;

  // Normalize the phi angle to a range between -pi and pi.
  while (y(1) > M_PI) {
    y(1) -= 2 * M_PI;
  }
  while (y(1) < -M_PI) {
    y(1) += 2 * M_PI;
  }
  KFHelper(y);
}

void KalmanFilter::KFHelper(const VectorXd &y) {
  // KF Measurement update step
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
