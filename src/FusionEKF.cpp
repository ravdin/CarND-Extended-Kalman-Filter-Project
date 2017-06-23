#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //set the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

  ekf_.Q_ = MatrixXd(4, 4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    ekf_.x_ = VectorXd(4);
    float x, y, vx, vy;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      x = rho * cos(phi);
      y = rho * sin(phi);
      vx = rho_dot * cos(phi);
      vy = rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x = measurement_pack.raw_measurements_[0];
      y = measurement_pack.raw_measurements_[1];
      vx = vy = 0;
    }

    ekf_.x_ << x, y, vx, vy;

    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //set the acceleration noise components
 	float noise_ax = 5;
 	float noise_ay = 5;

  //compute the time elapsed between the current and previous measurements
 	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
 	previous_timestamp_ = measurement_pack.timestamp_;

 	float dt2 = pow(dt, 2);
 	float dt3 = pow(dt, 3) / 2;
 	float dt4 = pow(dt, 4) / 4;

 	//1. Modify the F matrix so that the time is integrated
 	ekf_.F_(0, 2) = dt;
 	ekf_.F_(1, 3) = dt;
 	//2. Set the process covariance matrix Q
 	ekf_.Q_ << dt4 * noise_ax, 0, dt3 * noise_ax, 0,
 	          0, dt4 * noise_ay, 0, dt3 * noise_ay,
 	          dt3 * noise_ax, 0, dt2 * noise_ax, 0,
 	          0, dt3 * noise_ay, 0, dt2 * noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
  }

  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
