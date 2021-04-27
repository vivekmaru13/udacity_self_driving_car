#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  /**
   * TODO: Calculate the RMSE here.
   */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size())
    {
		std::cout << "Estimation and groun truth data has to be the same size." << std::endl;
		return rmse;
	}
	if (estimations.size() == 0)
    {
      std::cout << "Estimation data can not be zero. Invalid Estimation data" << std::endl;
      return rmse;
    }
	// accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i)
    {
		VectorXd residual = estimations[i] - ground_truth[i];
		VectorXd residual_square = residual.array()*residual.array();
		rmse += residual_square;
	}

	// mean
	rmse = rmse/estimations.size();

	// squared root
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd H_jacobian(3,4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-computation from Lecture answers
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001)
    {
		std::cout << "Either px or py is too small, Division by Zero error will occur. Check Jacobian Calculation." << std::endl;
		return H_jacobian;
	}

	//Jacobian matrix
	H_jacobian << 	(px/c2), (py/c2), 0, 0,
			       -(py/c1), (px/c1), 0, 0,
			        py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3,  px/c2, py/c2;

	return H_jacobian;
}
