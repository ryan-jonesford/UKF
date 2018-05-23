#ifndef TESTS_H
#define TESTS_H

#include <iostream>
#include <math.h>
#include "ekf.h"
#include "ukf.h"
#include "tools.h"
#include <fstream>

class Tests {
    public:
    bool test_PredictUKF( void );
    bool test_UpdateRadar( void );
};

#endif /* TESTS_H */