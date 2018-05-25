#ifndef UKF_H
#define UKF_H

#include <fstream>
#include <string>
#include <vector>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF : public KalmanFilter {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  MatrixXd Xsig_aug_;
  MatrixXd Xsig_pred_;
  VectorXd weights;
  long long time_us_;
  double std_a_;
  double std_yawdd_;
  double std_laspx_;
  double std_laspy_;
  double std_radr_;
  double std_radphi_;
  double std_radrd_;

  VectorXd weights_;

  int n_x_;
  int n_aug_;
  int n_z_;
  double lambda_;
  double srt_lambda_n_aug_;
  bool is_initalized_;

  //

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage& measurement_pack);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param dt Time between k and k+1 in s
   */
  void PredictUKF(double dt);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(VectorXd z);

  /**
   * Returns the type of kalman filter implemented
   */
  const char* get_kf_type(void);
};

#endif /* UKF_H */
