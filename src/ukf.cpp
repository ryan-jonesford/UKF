#include "Inc/ukf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  verbose_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial process noise vector
  u_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  F_ = MatrixXd(5,5);
  F_ <<  1, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 1, 0, 0,
         0, 0, 0, 1, 0, 
         0, 0, 0, 0, 1; 

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  srt_lambda_n_aug_ = sqrt(lambda_ + n_aug_);

  // Initalize the predicted and generated sigma points matrix
  Xsig_pred_ = MatrixXd::Zero( n_aug_, 2 * n_aug_ + 1 );
  Xsig_gen_ =  MatrixXd::Zero( n_aug_, 2 * n_aug_ + 1 );
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // get time step in seconds
  double dt_1 = get_dt(measurement_pack);

  // update the F matrix for the prediction step
  F_(0,2) = (1/x_(3))*(sin(x_(3)+x_(4)*dt_1) - sin(x_(3)));
  F_(1,2) = (1/x_(3))*(-cos(x_(3)+x_(4)*dt_1) + cos(x_(3)));
  F_(4,4) = dt_1;

  double dt_2 = dt_1  * dt_1;
  //update the process noise matrix
  u_ << .5 * dt_2 * cos(x_(3)),
        .5 * dt_2 * sin(x_(3)),
        dt_1 * x_(2),
        .5 * dt_2 * x_(4),
        dt_1 * x_(4);
  
  PredictUKF(dt_1);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::PredictUKF(double dt) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Predict state vecotr and covariance matrix
  // Predict();

  verbosity("creating augmented mean vector");
  VectorXd x_aug = VectorXd::Zero(n_aug_);

  verbosity("setting x_ to head of x_aug");
  x_aug.head(n_x_) = x_;

  verbosity("Creating augmented state covariance matrix");
  MatrixXd A = MatrixXd::Zero( n_aug_ ,n_aug_ );

  verbosity("setting P_ to top left corner of A");
  A.topLeftCorner( n_x_, n_x_ ) = P_;

  verbosity("Creating and setting Q_aug to bottom right corner of A");
  MatrixXd Q_aug = MatrixXd::Zero(2,2);
  Q_aug(0,0) = std_a_ * std_a_;
  Q_aug(1,1) = std_yawdd_ * std_yawdd_;
  A.block( n_x_, n_x_, 2, 2 ) = Q_aug;

  verbosity("Getting the square root of A");
  A = A.llt().matrixL();

  verbosity("Generating the sigma points");  
  verbosity("Setting x_ to the first column of Xsig_gen_");
  Xsig_gen_.col(0).head(n_x_)  = x_;

  verbosity("Setting the remaining sigma poitns to Xsig_gen_");
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_gen_.col(i+1)        = x_aug + srt_lambda_n_aug_ * A.col(i);
    Xsig_gen_.col(i+1+n_aug_) = x_aug - srt_lambda_n_aug_ * A.col(i);
  } 

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
