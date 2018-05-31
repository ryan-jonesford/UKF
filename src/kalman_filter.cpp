#include "Inc/kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/**
 * Name: Predict
 * Return: None
 * Description: Implements Kalman Filter Prediction
 **/
void KalmanFilter::Predict() {
  x_ = F_ * x_ + u_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

/**
 * Name: Update
 * Return: None
 * Description: Updates the state by using standard Kalman Filter equations
 **/
void KalmanFilter::Update(const VectorXd &z) {
  // clang-format off
  VectorXd z_pred = H_ * x_;
  VectorXd y      = z - z_pred;
  MatrixXd Ht     = H_.transpose();
  MatrixXd S      = H_ * P_ * Ht + R_;
  MatrixXd PHt    = P_ * Ht;
  MatrixXd K      = PHt * S.inverse();
  // clang-format on

  // new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  if( print_nis_ ){
    tools.track_nis(tools.LASER, z_pred, z, S);
  }
}

/**
 * Name: get_dt
 * Return: change in time in seconds
 * Description: get the time step and check for out of bounds before returning
 **/
double KalmanFilter::get_dt(const MeasurementPackage &measurement_pack) {
  // Update time, dt expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // If dt_1 is out of bounds, give warning
  // if (dt <= 0 || dt > MAX_DT) {
    // cerr << "KalmanFilter::get_dt: Something went wrong... dt = " << dt
  //        << endl;
  // }
  return dt;
}

/**
 * Name: verbosity
 * Return: None
 * Description: Prints @param statement to the console if verbose_ is true
 **/
void KalmanFilter::verbosity(const char *statement) {
  if (verbose_) {
    cout << statement << endl;
  }
}