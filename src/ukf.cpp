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

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Radar measurement direction: r, phi and r_dot
  n_z_ = 3;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  srt_lambda_n_aug_ = sqrt(lambda_ + n_aug_);

  //set weights
  weights = VectorXd::Zero(2*n_aug_+1);
  weights(0) = lambda_/(lambda_+n_aug_);
  for(int i = 1; i < 2*n_aug_+1; ++i)
  {
      weights(i) = 1/(2*(lambda_ + n_aug_));
  }

  // Initalize the predicted and augmented sigma points matrix
  Xsig_pred_ = MatrixXd::Zero( n_x_, 2 * n_aug_ + 1 );
  Xsig_aug_ =  MatrixXd::Zero( n_aug_, 2 * n_aug_ + 1 );
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
  
  PredictUKF(dt_1);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::PredictUKF(double dt_1) {
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
  verbosity("Setting x_ to the first column of Xsig_aug_");
  Xsig_aug_.col(0).head(n_x_)  = x_;

  verbosity("Setting the remaining sigma poitns to Xsig_aug_");
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i+1)        = x_aug + srt_lambda_n_aug_ * A.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - srt_lambda_n_aug_ * A.col(i);
  } 

  double dt_2 = dt_1  * dt_1;

  verbosity("Making prediction for the Sigma Points");
  for( int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
      // get first 5 rows of column i set = to vector x
      VectorXd x       = Xsig_aug_.block(0,i,n_x_,1);
      VectorXd x_pred  = VectorXd(n_x_);
      VectorXd x_noise = VectorXd(n_x_);
      // Avoiding divide by zero
      if( x(4) ){
          x_pred  << (x(2)/x(4))*(sin(x(3)+x(4)*dt_1) - sin(x(3))),
                     (x(2)/x(4))*(-cos(x(3)+x(4)*dt_1) + cos(x(3))),
                     0,
                     dt_1 * x(4),
                     0;
      }
      else{
          x_pred  << x(2) * cos(x(3)) * dt_1,
                     x(2) * sin(x(3)) * dt_1,
                     0,
                     0,
                     0;
      }
      x_noise     <<.5 * dt_2 * cos(x(3)) * Xsig_aug_(n_x_,i),
                    .5 * dt_2 * sin(x(3)) * Xsig_aug_(n_x_,i),
                    dt_1 * Xsig_aug_(n_x_,i),
                    .5 * dt_2 * Xsig_aug_(n_x_+1,i),
                    dt_1 * Xsig_aug_(n_x_+1,i);
      // Add the vectors up and put them into column i of predicted sigma pts          
      Xsig_pred_.col(i) = x+x_pred+x_noise; 
  }

  x_.setZero();
  P_.setZero();
  verbosity("Predicting the state mean");
  for( int i = 0; i < 2*n_aug_+1; ++i)
  {
      x_ += Xsig_pred_.col(i)*weights(i);
  }
  
  verbosity("Predicting state covariance matrix");
  for( int i = 0; i < 2*n_aug_+1; ++i)
  {
      VectorXd F = VectorXd(n_x_);
      F          = Xsig_pred_.col(i)-x_;
      //angle normalization
      F(3) = tools.normalizePhi(F(3));
      P_ += weights(i) * F * F.transpose();
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

  VectorXd z = VectorXd(n_z_);
  z = meas_package.raw_measurements_;
 
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  verbosity("transform sigma points into measurement space");
  for(int i = 0; i < 2 * n_aug_ +1;++i)
  {
    Zsig.col(i) = tools.CartesianToPolar(Xsig_pred_.col(i));
  }

  verbosity("calculate mean predicted measurement");
  z_pred.setZero();
  for(int i = 0; i < 2 * n_aug_ +1;++i)
  {
      z_pred += weights(i) *  Zsig.col(i);
  }

  verbosity("calculate innovation covariance matrix, S");
  MatrixXd S = MatrixXd::Zero(n_z_,n_z_);
  MatrixXd R = MatrixXd::Zero(n_z_,n_z_);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  for(int i = 0; i < 2 * n_aug_ +1;++i)
  {
      VectorXd s = VectorXd::Zero(3);
      s = Zsig.col(i) - z_pred;
      //angle normalization
      s(1) = tools.normalizePhi(s(1));
      S   += weights(i)*s*s.transpose();
  }
  S += R;

  verbosity("calculate cross correlation matrix");
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z_);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
      VectorXd t1 = VectorXd::Zero(3);
      VectorXd t2 = VectorXd::Zero(3);

      t1 = Xsig_pred_.col(i)-x_;
      t2 = Zsig.col(i) - z_pred;
      //angle normalization
      t1(3) = tools.normalizePhi(t1(3));
      t2(1) = tools.normalizePhi(t2(1));
      Tc   += weights(i)*t1*t2.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();
  //update state mean and covariance matrix
  MatrixXd z_diff = z-z_pred;
  //angle normalization
  z_diff(1) = tools.normalizePhi(z_diff(1));
  x_        = x_ + K*z_diff;
  P_        = P_ - K*S*K.transpose();
}
