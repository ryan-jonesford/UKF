#include "Inc/ekf.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
EKF::EKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

  // hard code noise for now
  noise_ax_ = 9, noise_ay_ = 9;

  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
  P_ = MatrixXd::Zero(4, 4);
  Q_ = MatrixXd::Zero(4, 4);

  u_ = VectorXd::Zero(4);
}

/**
 * Destructor.
 */
EKF::~EKF() {}

/**
 * Name: ProcessMeasurement
 * Return: None
 * Description: Updates the state predictions from measurements
 **/
void EKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    x_ = VectorXd::Ones(4);
    // set timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // Use a switch statement for sensor type switching
    // the {} in the case statements are strictly necessary, but are a useful
    // reminder for the future if you want to declare and use variables in them.
    switch (measurement_pack.sensor_type_) {
      case (MeasurementPackage::RADAR): {
        VectorXd px_py =
            tools.PolarToCartesian(measurement_pack.raw_measurements_);
        x_(0) = px_py(0);
        x_(1) = px_py(1);
        P_ << 500, 0, 0, 0, 0, 500, 0, 0, 0, 0, 500, 0, 0, 0, 0, 500;
        break;
      }
      case (MeasurementPackage::LASER): {
        x_(0) = measurement_pack.raw_measurements_(0);
        x_(1) = measurement_pack.raw_measurements_(1);
        P_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;
        break;
      }
      default:
        cerr << "EKF::ProcessMeasurement: Invalid Measurment device" << endl;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // get time step in seconds
  float dt_1 = get_dt(measurement_pack);

  // the state transition matrix F
  F_(0, 2) = dt_1;
  F_(1, 3) = dt_1;

  // Update the process noise covariance matrix
  float dt_2 = dt_1 * dt_1;
  float dt_3 = dt_2 * dt_1;
  float dt_4 = dt_3 * dt_1;

  Q_(0, 0) = (dt_4 / 4.0) * noise_ax_;
  Q_(0, 2) = (dt_3 / 2.0) * noise_ax_;
  Q_(1, 1) = (dt_4 / 4.0) * noise_ay_;
  Q_(1, 3) = (dt_3 / 2.0) * noise_ay_;
  Q_(2, 0) = (dt_3 / 2.0) * noise_ax_;
  Q_(2, 2) = (dt_2 / 1.0) * noise_ax_;
  Q_(3, 1) = (dt_3 / 2.0) * noise_ay_;
  Q_(3, 3) = (dt_2 / 1.0) * noise_ay_;

  Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Different update routines based on sensor type
  switch (measurement_pack.sensor_type_) {
    case (MeasurementPackage::RADAR): {
      R_ = R_radar_;
      H_ = tools.CalculateJacobian(x_);
      UpdateEKF(measurement_pack.raw_measurements_);
      break;
    }
    case (MeasurementPackage::LASER): {
      R_ = R_laser_;
      H_ = H_laser_;
      Update(measurement_pack.raw_measurements_);
      break;
    }
    default:
      cerr << "EKF::ProcessMeasurement: Invalid Measurment device" << endl;
  }
}

/**
 * Name: UpdateEKF
 * Return: None
 * Description: Updates the state by using Extended Kalman Filter equations
 **/
void EKF::UpdateEKF(const VectorXd &z) {
  VectorXd z_pred = tools.CartesianToPolar(x_);
  ;
  MatrixXd y = z - z_pred;
  y(1) = tools.normalizePhi(y(1));
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * S.inverse();

  // new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

const char *EKF::get_kf_type(void) { return "EKF"; }