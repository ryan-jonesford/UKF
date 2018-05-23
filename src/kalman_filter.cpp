#include "Inc/kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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

/**
 * Name: Predict
 * Return: None
 * Description: Implements Kalman Filter Prediction
**/
void KalmanFilter::Predict() {
  x_ = F_ * x_ + u_;
  P_ = F_*P_*F_.transpose() + Q_;
}

/**
 * Name: Update
 * Return: None
 * Description: Updates the state by using standard Kalman Filter equations
**/
void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd z_pred = H_ * x_;
	VectorXd y      = z - z_pred;
	MatrixXd Ht     = H_.transpose();
	MatrixXd S      = H_ * P_ * Ht + R_;
	MatrixXd PHt    = P_ * Ht;
	MatrixXd K      = PHt * S.inverse();

	//new estimate
	x_ 			    = x_ + (K * y);
	int x_size 	= x_.size();
	MatrixXd I 	= MatrixXd::Identity(x_size, x_size);
	P_ 			    = (I - K * H_) * P_;
}

/**
 * Name: get_dt
 * Return: change in time in seconds
 * Description: get the time step and check for out of bounds before returning
**/
double KalmanFilter::get_dt(const MeasurementPackage &measurement_pack){
  //Update time, dt expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // If dt_1 is out of bounds, end program
  if( dt <= 0 || dt > MAX_DT )
  {
    cerr << "EKF::ProcessMeasurement: Something went wrong... dt = "
         << dt << endl;
    exit(1);
  }
  return dt;
}

void KalmanFilter::verbosity(const char * statement)
{
	if (verbose_)
	{
		cout<<statement<<endl;
	}
}