#include <iostream>
#include "Inc/tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

/**
 * Name: Tools
 * Description: Tools class creates helper functions used for
 * 				implementing and evaluating Kalman filters
**/
Tools::Tools() {}

/**
 * Name: Tools
 * Description: Tools class Destructor
**/
Tools::~Tools() {}

/**
 * Name: CalculateRMSE
 * Return: Root mean squared values expressed as VectorXd
 * Description: Calculates the RMSE
**/
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
    unsigned int vector_size = estimations.size();
 	unsigned int vectorXd_size = estimations[0].size();
	VectorXd rmse = VectorXd::Zero(vectorXd_size);

	// the estimation vector size should not be zero
	// the estimation vector size should equal ground truth vector size
	if( !vector_size ||
		vector_size != ground_truth.size())
 	{
		cout << "Tools::CalculateRMSE: Invalid estimation "
				"or ground_truth Vector" << endl;
		return rmse;
	}

	//accumulate squared residuals
	VectorXd residuals = VectorXd::Zero(vectorXd_size);
	VectorXd resid(vectorXd_size);
	for(unsigned int i=0; i < vector_size; ++i){
		resid = (estimations[i] - ground_truth[i]);
	    resid = resid.array()*resid.array();
        residuals += resid;
	}

	//calculate the mean
	rmse = residuals/vector_size;

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

/**
 * Name: CalculateJacobian
 * Return: Jacobian matrix expressed as MatrixXd
 * Description: Calculates a Jacobian matrix from a given state
**/
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj = MatrixXd::Zero(3,4);
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);
	double px2py2 = px*px+py*py;
	double px2py2_sqrt = sqrt(px2py2);

	//check division by (or close to) zero
	if( fabs(px2py2) < 0.0001 )
  	{
		cout << "Tools::CalculateJacobian: Division by Zero detected!! Bailing out." << endl;
		return Hj;
	}
	
	//compute the Jacobian matrix
	Hj(0,0) = px/px2py2_sqrt;
	Hj(0,1) = py/px2py2_sqrt;
	Hj(1,0) = -py/px2py2;
	Hj(1,1) = px/px2py2;
	Hj(2,0) = (py*(vx*py-vy*px))/(pow(px2py2,(3.0/2.0)));
	Hj(2,1) = (px*(vy*px-vx*py))/(pow(px2py2,(3.0/2.0)));
	Hj(2,2) = px/px2py2_sqrt;
	Hj(2,3) = py/px2py2_sqrt;

	return Hj;
}

/**
 * Name: PolarToCartesian
 * Return: VectorXd in cartesian coordinates
 * Description: Convert Polar coords to cartesian
**/
VectorXd Tools::PolarToCartesian(const VectorXd& polar){
	double rho = polar(0);
	double phi = polar(1);
	double px  = cos(phi) + rho;
	double py  = sin(phi) + rho;

    VectorXd cartesian = VectorXd(2);
	cartesian(0) 	   = px;
	cartesian(1) 	   = py;
	return cartesian;
}

/**
 * Name: CartesianToPolar
 * Return: VectorXd in Polar coordinates
 * Description: Convert cartesian coords to Polar
**/
VectorXd Tools::CartesianToPolar(const VectorXd& cartesian){
	double px 	= 0, py	  = 0;
	double vx	= 0, vy   = 0, v = 0;
	double psi 	= 0;
	
	
	// grab the cartesian coords
	if ( cartesian.size() == 4 ) //EKF
	{
		px = cartesian(0);
		py = cartesian(1);
		vx = cartesian(2);
		vy = cartesian(3);
	}
	else if(cartesian.size() == 5 ) // UKF
	{
		px   = cartesian(0);
		py   = cartesian(1);
		v    = cartesian(2);
		psi  = cartesian(3);
		// psid = cartesian(4) ...but ignore it, it's not used
	}
	else
	{
		cerr << "Tools::CartesianToPolar: Cartesian vector "
		        "has invalid number of elemnts!! Bailing out." << endl;
		exit(1);
	}

	// setup to convert to polar
	double px2py2 = px*px+py*py;
	//check division by (or close to) zero
	if( fabs(px2py2) < 0.0001 )
  	{
		cerr << "Tools::CartesianToPolar: Division by Zero detected!! Bailing out." << endl;
		exit(1);
	}
	double sqrt_px2_py2 = sqrt(px2py2);

	// take atan, normalized between -pi and pi
	double atan__py_div_px = atan2(py,px);
	
	// populate the polar vector and return
	VectorXd polar = VectorXd(3);
	if ( cartesian.size() == 4 ) //EKF
	{
		polar << sqrt_px2_py2,
				 atan__py_div_px,
				 ( px*vx + py*vy ) / sqrt_px2_py2;
	}
	else // UKF
	{
		polar << sqrt_px2_py2,
			     atan__py_div_px,
			     ( px*cos(psi)*v + py*sin(psi)*v ) / sqrt_px2_py2;
	}

	return polar;
}

double Tools::normalizePhi(double phi){
	while (phi < -M_PI){
		phi += 2 * M_PI;
	}
	while ( phi > M_PI ){
		phi -= 2 * M_PI;
	}
	if( fabs(phi) > M_PI )
	{
		cout << "Tools::normalizePhi: phi outside of -pi to pi: "
			 << phi << "\nBailing out." << endl;
		exit(1);
	}
	return phi;
}