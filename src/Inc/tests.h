#ifndef TESTS_H
#define TESTS_H

#include <math.h>
#include <fstream>
#include <iostream>
#include "ekf.h"
#include "tools.h"
#include "ukf.h"

class Tests {
   public:
    bool test_PredictUKF(void);
    bool test_UpdateRadar(void);
};

#endif /* TESTS_H */