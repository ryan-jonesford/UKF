#ifndef TOOLS_H_
#define TOOLS_H_
#include <cmath>
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
   public:
    /**
     * Constructor.
     */
    Tools();

    /**
     * Destructor.
     */
    virtual ~Tools();

    /**
     * A helper method to calculate RMSE.
     */
    VectorXd CalculateRMSE(const vector<VectorXd>& estimations,
                           const vector<VectorXd>& ground_truth);

    /**
     * A helper method to calculate Jacobians.
     */
    MatrixXd CalculateJacobian(const VectorXd& x_state);

    /**
     * A helper method to convert Polar coords to cartesian.
     */
    VectorXd PolarToCartesian(const VectorXd& polar);

    /**
     * A helper method to convert cartesian coords to Polar.
     */
    VectorXd CartesianToPolar(const VectorXd& cartesian);

    /**
     * A helper method to normalize an angle between -Pi and Pi
     */
    double normalizePhi(double phi);

    enum Sensor_type {
        RADAR,
        LASER,
    };

    /**
     * A method to track nis values and print them to the console
     */
    void track_nis(Sensor_type sensor_type, const VectorXd& pred,
                   const VectorXd& actual, const MatrixXd& S);

   private:
    int radar_samp_count_;
    int laser_samp_count_;
    double nis_r_lower_than_95pcnt_;
    double nis_l_lower_than_95pcnt_;
};

#endif /* TOOLS_H_ */
