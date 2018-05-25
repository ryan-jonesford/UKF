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
  // Make this true to print an anoying amount of debug statements to the
  // console
  verbose_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  /*****
   * DO NOT MODIFY measurement noise values below these are provided by the
   *sensor manufacturer.
   *****/
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
  /*****
   * DO NOT MODIFY measurement noise values above these are provided by the
   *sensor manufacturer.
   ****/

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = .8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .75;

  // initial state vector
  x_ = VectorXd(5);

  // initial process noise vector
  u_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // clang-format off
  // measurement matrix
  H_ = MatrixXd(2,5);
  H_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0;

  // Laser measurement covariance matrix
  R_ = MatrixXd(2,2);
  R_<< std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;
  // clang-format on

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Radar measurement direction: r, phi and r_dot
  n_z_ = 3;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  srt_lambda_n_aug_ = sqrt(lambda_ + n_aug_);

  // set weights
  weights = VectorXd::Zero(2 * n_aug_ + 1);
  weights(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights(i) = 1 / (2 * (lambda_ + n_aug_));
  }

  // Initalize the predicted and augmented sigma points matrix
  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
  Xsig_aug_ = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);

  print_nis_ = false;

  is_initalized_ = false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage& measurement_pack) {
  if (!is_initalized_) {
    // set timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // first measurement
    x_ = VectorXd::Ones(n_x_);

    switch (measurement_pack.sensor_type_) {
      case (MeasurementPackage::RADAR): {
        verbosity("Initalizing UKF with Radar measurement");
        VectorXd px_py =
            tools.PolarToCartesian(measurement_pack.raw_measurements_);
        x_(0) = px_py(0);
        x_(1) = px_py(1);
        // clang-format off
        P_ << 0.2, 0.0,   0.0,   0.0, 0.0,
              0.0, 0.009, 0.0,   0.0, 0.0,
              0.0, 0.0,   0.283, 0.0, 0.0,
              0.0, 0.0,   0.0,   2.0, 0.0,
              0.0, 0.0,   0.0,   0.0, 0.0;
        // clang-format on
        break;
      }
      case (MeasurementPackage::LASER): {
        verbosity("Initalizing UKF with Lasar measurement");
        x_(0) = measurement_pack.raw_measurements_(0);
        x_(1) = measurement_pack.raw_measurements_(1);
        // clang-format off
        P_ <<  0.075, 0.0,   0.0, 0.0,  0.0,
               0.0,   0.075, 0.0, 0.0,  0.0,
               0.0,   0.0,   2.7, 0.0,  0.0,
               0.0,   0.0,   0.0, 20.0, 0.0,
               0.0,   0.0,   0.0, 0.0,  20.0;
        // clang-format on
        break;
      }
      default:
        cerr << "Bad Sensor type" << endl;
        exit(1);
    }
    is_initalized_ = true;
    return;  // don't need to make predictions
  }
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // get time step in seconds
  double dt_1 = get_dt(measurement_pack);

  // Execute the predition step
  PredictUKF(dt_1);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Different update routines based on sensor type
  switch (measurement_pack.sensor_type_) {
    case (MeasurementPackage::RADAR): {
      if (use_laser_) {
        return;
      }
      verbosity("Updating UKF with Radar measurement");
      UpdateRadar(measurement_pack.raw_measurements_);
      break;
    }
    case (MeasurementPackage::LASER): {
      if (use_radar_) {
        return;
      }
      verbosity("Updating UKF with Lasar measurement");
      Update(measurement_pack.raw_measurements_);
      break;
    }
    default:
      cerr << "EKF::ProcessMeasurement: Invalid Measurment device" << endl;
  }
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
  MatrixXd A = MatrixXd::Zero(n_aug_, n_aug_);

  verbosity("setting P_ to top left corner of A");
  A.topLeftCorner(n_x_, n_x_) = P_;

  verbosity("Creating and setting Q_aug to bottom right corner of A");
  MatrixXd Q_aug = MatrixXd::Zero(2, 2);
  Q_aug(0, 0) = std_a_ * std_a_;
  Q_aug(1, 1) = std_yawdd_ * std_yawdd_;
  A.block(n_x_, n_x_, 2, 2) = Q_aug;

  verbosity("Getting the square root of A");
  A = A.llt().matrixL();

  verbosity("Generating the sigma points");
  verbosity("Setting x_ to the first column of Xsig_aug_");
  Xsig_aug_.col(0).head(n_x_) = x_;

  verbosity("Setting the remaining sigma poitns to Xsig_aug_");
  for (int i = 0; i < n_aug_; i++) {
    // clang-format off
    Xsig_aug_.col(i + 1) = x_aug + srt_lambda_n_aug_ * A.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - srt_lambda_n_aug_ * A.col(i);
    // clang-format ON
  }

  double dt_2 = dt_1 * dt_1;

  verbosity("Making prediction for the Sigma Points");
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // get first 5 rows of column i set = to vector x
    VectorXd x = Xsig_aug_.block(0, i, n_x_, 1);
    VectorXd x_pred = VectorXd(n_x_);
    VectorXd x_noise = VectorXd(n_x_);
    // Avoiding divide by zero
    if (x(4)) {
      // clang-format off
      x_pred << (x(2) / x(4)) * (sin(x(3) + x(4) * dt_1) - sin(x(3))),
          (x(2) / x(4)) * (-cos(x(3) + x(4) * dt_1) + cos(x(3))), 0,
          dt_1 * x(4), 0;
      // clang-format on
    } else {
      x_pred << x(2) * cos(x(3)) * dt_1, x(2) * sin(x(3)) * dt_1, 0, 0, 0;
    }
    x_noise << .5 * dt_2 * cos(x(3)) * Xsig_aug_(n_x_, i),
        .5 * dt_2 * sin(x(3)) * Xsig_aug_(n_x_, i), dt_1 * Xsig_aug_(n_x_, i),
        .5 * dt_2 * Xsig_aug_(n_x_ + 1, i), dt_1 * Xsig_aug_(n_x_ + 1, i);
    // Add the vectors up and put them into column i of predicted sigma pts
    Xsig_pred_.col(i) = x + x_pred + x_noise;
  }

  x_.setZero();
  P_.setZero();
  verbosity("Predicting the state mean");
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x_ += Xsig_pred_.col(i) * weights(i);
  }

  verbosity("Predicting state covariance matrix");
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd F = VectorXd(n_x_);
    F = Xsig_pred_.col(i) - x_;
    // angle normalization
    F(3) = tools.normalizePhi(F(3));
    P_ += weights(i) * F * F.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(VectorXd z) {
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);

  verbosity("transform sigma points into measurement space");
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    Zsig.col(i) = tools.CartesianToPolar(Xsig_pred_.col(i));
  }

  verbosity("calculate mean predicted measurement");
  z_pred.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred += weights(i) * Zsig.col(i);
  }

  verbosity("calculate innovation covariance matrix, S");
  MatrixXd S = MatrixXd::Zero(n_z_, n_z_);
  MatrixXd R = MatrixXd::Zero(n_z_, n_z_);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd s = VectorXd::Zero(3);
    s = Zsig.col(i) - z_pred;
    // angle normalization
    s(1) = tools.normalizePhi(s(1));
    S += weights(i) * s * s.transpose();
  }
  S += R;

  verbosity("calculate cross correlation matrix");
  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z_);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    VectorXd t1 = VectorXd::Zero(3);
    VectorXd t2 = VectorXd::Zero(3);

    t1 = Xsig_pred_.col(i) - x_;
    t2 = Zsig.col(i) - z_pred;
    // angle normalization
    t1(3) = tools.normalizePhi(t1(3));
    t2(1) = tools.normalizePhi(t2(1));
    Tc += weights(i) * t1 * t2.transpose();
  }
  verbosity("calculate Kalman gain K");
  MatrixXd K = Tc * S.inverse();
  verbosity("update state mean and covariance matrix");
  MatrixXd z_diff = z - z_pred;
  // angle normalization
  z_diff(1) = tools.normalizePhi(z_diff(1));
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate the radar NIS.
  if (print_nis_) {
    tools.track_nis(tools.RADAR, z_pred, z, S);
  }
}

/**
 * Returns the type of Kalman Filter that's been implemented
 */
const char* UKF::get_kf_type(void) { return "UKF"; }
