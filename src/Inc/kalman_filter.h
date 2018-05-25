#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"
#include "measurement_package.h"

static const int MAX_DT = 120; // 2min

class KalmanFilter {
public:

    // state vector
    Eigen::VectorXd x_;

    // process noise vector
    Eigen::VectorXd u_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    /** 
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
    Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    virtual void Predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z);

    /**
     * Returns the type of kalman filter implemented
     */
    virtual const char* get_kf_type( void ){ return " "; }

    /**
     * Run the whole flow of the Kalman Filter from here. Must be implemented by
     * children classes
     */
    virtual void ProcessMeasurement(const MeasurementPackage &measurement_pack){};

    // if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    // if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    // if true, prints to console what it is doing
    bool verbose_;

    // if true, prints the nis results to the console
    bool print_nis_;
    

  protected:
    Tools tools;

    //* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    /**
     * Get the time step and check for out of bounds before returning
     */
    double get_dt(const MeasurementPackage &measurement_pack);

    void verbosity(const char *statement);
};

#endif /* KALMAN_FILTER_H_ */
