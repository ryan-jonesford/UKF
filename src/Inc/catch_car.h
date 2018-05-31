#ifndef CATCH_CAR_H
#define CATCH_CAR_H

#include <vector>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class carCatcher {
   public:
    /**
     * Constructor
     */
    carCatcher();
    /**
     * Destructor
     */
    ~carCatcher();
    VectorXd first_loc;
    bool first_loc_recorded;
    bool first_loc_reached;

    /**
     * capture
     * @param hunter: the position and heading of the chase vehicle
     * @param prey: the positing, velocity, heading and change in heading of the
     *        vehicle being chased
     */
    VectorXd capture(VectorXd hunter, VectorXd prey);

    /**
     * head_to
     * @param x: X-position to go to
     * @param y: Y-position to go to
     * @param current: current location
     */
    VectorXd head_to(double x, double y, VectorXd current);
    Tools tools;
};

#endif /* CATCH_CAR_H */