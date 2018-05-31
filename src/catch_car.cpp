#include "Inc/catch_car.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 *  Constructor
 **/
carCatcher::carCatcher() {
    first_loc_reached = false;
    first_loc_recorded = false;
    first_loc = VectorXd::Zero(5);
}
/**
 *  Destructor
 **/
carCatcher::~carCatcher() {}

/**
 * Name: capture
 * Return: heading and distance to point expressed as a VectorXd
 * Description: determines the heading and the distance to a target given the
 *              current location and the target
 **/
VectorXd carCatcher::capture(VectorXd hunter, VectorXd prey) {
    VectorXd ret_vect = VectorXd::Zero(2);
    double hunter_x = hunter[0];
    double hunter_y = hunter[1];

    double prey_x = prey[0];
    double prey_y = prey[1];

    // go to where prey was first spotted
    if (!first_loc_recorded) {
        first_loc = prey;
        first_loc_recorded = true;
    }
    if (!first_loc_reached) {
        ret_vect = head_to(first_loc[0], first_loc[1], hunter);
        if (ret_vect[1] < .1) {
            first_loc_reached = true;
        }
    } else {
        // if prey is close, head towards it
        if (sqrt(pow(prey_x - hunter_x, 2) + pow(prey_y - hunter_y, 2)) < 2) {
            ret_vect = head_to(prey[0], prey[1], hunter);
        } else {
            // otherwise stare at it and wait in the shadows...
            ret_vect = head_to(prey[0], prey[1], hunter);
            ret_vect[1] = 0;
        }
    }
    return ret_vect;
}

/**
 * Name: head_to
 * Return: heading and distance to point expressed as a VectorXd
 * Description: determines the heading and the distance to a point given the
 *              current location and the target point
 **/
VectorXd carCatcher::head_to(double x, double y, VectorXd current) {
    VectorXd ret_vector = VectorXd(2);
    double dist_to_point =
        sqrt(pow(x - current[0], 2) + pow(y - current[1], 2));
    // heading to taret
    double heading = atan2(y - current[1], x - current[0]);
    while (heading > M_PI) heading -= 2. * M_PI;
    while (heading < -M_PI) heading += 2. * M_PI;
    // turn towards the target
    heading = heading - current[2];
    while (heading > M_PI) heading -= 2. * M_PI;
    while (heading < -M_PI) heading += 2. * M_PI;

    ret_vector << heading, dist_to_point;
    return ret_vector;
}