#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
  	rmse << 0,0,0,0;

  	// check the validity of the following inputs:
  	//  * the estimation vector size should not be zero
  	//  * the estimation vector size should equal ground truth vector size
  	if(estimations.size() != ground_truth.size()
  			|| estimations.size() == 0){
  		cout << "Invalid estimation or ground_truth data" << endl;
  		return rmse;
  	}

  	//accumulate squared residuals
  	for(unsigned int i=0; i < estimations.size(); ++i){

  		VectorXd residual = estimations[i] - ground_truth[i];

  		//coefficient-wise multiplication
  		residual = residual.array()*residual.array();
  		rmse += residual;
  	}

  	//calculate the mean
  	rmse = rmse/estimations.size();

  	//calculate the squared root
  	rmse = rmse.array().sqrt();

  	//return the result
  	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE

	//check division by zero
	if (px == 0 || py == 0) {
	    cout << "Error - Division by zero";
	    return Hj;
	}

	//compute the Jacobian matrix
	float c1 = pow(px, 2) + pow(py, 2);
	float c2 = vy * px - vx * py;
	Hj << px / sqrt(c1), py / sqrt(c1), 0, 0,
	      -py / c1, px / c1, 0, 0,
	      py * c2 / pow(c1, 1.5), px * c2 / pow(c1, 1.5), px / sqrt(c1), py / sqrt(c1);

	return Hj;
}
