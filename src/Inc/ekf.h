#ifndef EKF_H_
#define EKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "kalman_filter.h"

class EKF : public KalmanFilter {
   public:
    /**
     * Constructor.
     */
    EKF();

    /**
     * Destructor.
     */
    virtual ~EKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     **/
    void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd &z);

    /**
     * Returns the type of kalman filter implemented
     */
    const char *get_kf_type(void);

   private:
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;

    // Process and measurement noise
    int noise_ax_, noise_ay_;
};

#endif /* EKF_H_ */
