#include "Inc/tests.h"

using namespace std;

bool Tests::test_PredictUKF( void ){
    bool passed = true;
    UKF ukf;
    ukf.verbose_ = true;
    ukf.x_ <<   5.7441,
                1.3800,
                2.2049,
                0.5015,
                0.3528; 
    ukf.P_  <<   0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
                -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
                 0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
                -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
                -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
    ukf.std_a_ = 0.2;          
    ukf.std_yawdd_ = 0.2;
    cout << "***running PredictUKF***"<<endl;
    ukf.PredictUKF(.1);

    MatrixXd expected_x_sig_aug = MatrixXd(7,15);
    expected_x_sig_aug << 
        5.7441,5.85768,5.7441,5.7441,5.7441,5.7441,5.7441,5.7441,5.63052,5.7441,5.7441,5.7441,5.7441,5.7441,5.7441,
        1.38,1.34566,1.52806,1.38,1.38,1.38,1.38,1.38,1.41434,1.23194,1.38,1.38,1.38,1.38,1.38,
        2.2049,2.28414,2.24557,2.29582,2.2049,2.2049,2.2049,2.2049,2.12566,2.16423,2.11398,2.2049,2.2049,2.2049,2.2049,
        0.5015,0.44339,0.631886,0.516923,0.595227,0.5015,0.5015,0.5015,0.55961,0.371114,0.486077,0.407773,0.5015,0.5015,0.5015,
        0.3528,0.299973,0.462123,0.376339,0.48417,0.418721,0.3528,0.3528,0.405627,0.243477,0.329261,0.22143,0.286879,0.3528,0.3528,
        0,0,0,0,0,0,0.34641,0,0,0,0,0,0,-0.34641,0,
        0,0,0,0,0,0,0,0.34641,0,0,0,0,0,0,-0.34641;
    
    cout << "***Comparing output***"<<endl;
    for(int i=0; i < 7; ++i)
    {
        for(int d=0; d < 15; ++d)
        {
            if( round(expected_x_sig_aug(i,d)*1000) != round(ukf.Xsig_aug_(i,d)*1000) )
            {
                cout<<"expected_x_sig_aug("<<i<<","<<d<<") does not equal ukf.Xsig_aug_("
                    <<i<<","<<d<<")"<<endl;
                cout << expected_x_sig_aug(i,d) << " != " << ukf.Xsig_aug_(i,d)<<endl;
                passed = false;
            }
        }
    }
    
    MatrixXd expected_x_sig_pred = MatrixXd(5,15);
    expected_x_sig_pred <<  5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
                            1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
                            2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
                            0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
                            0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;
    
    for(int i=0; i < 5; ++i)
    {
        for(int d=0; d < 15; ++d)
        {
            if( round(expected_x_sig_pred(i,d)*1000) != round(ukf.Xsig_pred_(i,d)*1000) )
            {
                cout<<"expected_x_sig_pred("<<i<<","<<d<<") does not equal ukf.Xsig_pred_("
                    <<i<<","<<d<<")"<<endl;
                cout << expected_x_sig_pred(i,d) << " != " << ukf.Xsig_pred_(i,d)<<endl;
                passed = false;
            }
        }
    }

    VectorXd state_vector_expected = VectorXd(5);
    state_vector_expected << 5.93445,
                             1.48885,
                             2.2049,
                             0.53678,
                             0.3528;

    MatrixXd covariance_expected = MatrixXd(5,5);
    covariance_expected <<   0.0054808, -0.00249899, 0.00340521, -0.0035741, -0.00309082, 
                            -0.00249899, 0.0110551, 0.00151803, 0.00990779, 0.00806653, 
                             0.00340521, 0.00151803, 0.0057998, 0.000780142, 0.000800107, 
                            -0.0035741, 0.00990779, 0.000780142, 0.0119239, 0.01125, 
                            -0.00309082, 0.00806653, 0.000800107, 0.01125, 0.0127;
    
    for(int i=0; i < 5; ++i)
    {
        if( round(state_vector_expected(i)*1000) != round(ukf.x_(i)*1000) )
        {
            cout<<"state_vector_expected("<<i<<") does not equal ukf.x_("
                <<i<<")"<<endl;
            cout << state_vector_expected(i) << " != " << ukf.x_(i)<<endl;
            passed = false;
        }
    }

    for(int i=0; i < 5; ++i)
    {
        for(int d=0; d < 5; ++d)
        {
            if( round(covariance_expected(i,d)*1000) != round(ukf.P_(i,d)*1000) )
            {
                cout<<"covariance_expected("<<i<<","<<d<<") does not equal ukf.P_("
                    <<i<<","<<d<<")"<<endl;
                cout << covariance_expected(i,d) << " != " << ukf.P_(i,d)<<endl;
                passed = false;
            }
        }
    }

    return passed;
}

bool Tests::test_UpdateRadar( void ){
    bool passed = true;
    UKF ukf;
    ukf.verbose_ = true;

    //set augmented dimension
    int n_aug = 7;

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    //define spreading parameter
    double lambda = 3 - n_aug;

    //set vector for weights
    VectorXd weights = VectorXd(2*n_aug+1);
      double weight_0 = lambda/(lambda+n_aug);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug+1; i++) {  //2n+1 weights
      double weight = 0.5/(n_aug+lambda);
      weights(i) = weight;
    }

    //create example matrix with predicted sigma points
    ukf.Xsig_pred_ <<
           5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
             1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
            2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
           0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
            0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

    //create example vector for predicted state mean
    ukf.x_ <<
       5.93637,
       1.49035,
       2.20528,
      0.536853,
      0.353577;

    //create example matrix for predicted state covariance
    ukf.P_ <<
    0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
    -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
    0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
 -  0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
 -  0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z);
    z <<
        5.9214,
        0.2187,
        2.0062;

    MeasurementPackage meas_package;
    meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = z;

    VectorXd expected_x =VectorXd(5);
    expected_x << 5.92276,
                  1.41823,
                  2.15593,
                  0.489274,
                  0.321338;
    MatrixXd expected_P = MatrixXd(5,5);
    expected_P <<  0.00361579, -0.000357881, 0.00208316, -0.000937196, -0.00071727,
                  -0.000357881, 0.00539867, 0.00156846, 0.00455342, 0.00358885,
                   0.00208316, 0.00156846, 0.00410651, 0.00160333, 0.00171811,
                  -0.000937196, 0.00455342, 0.00160333, 0.00652634, 0.00669436,
                  -0.00071719, 0.00358884, 0.00171811, 0.00669426, 0.00881797;

    //radar measurement noise standard deviation radius in m
    ukf.std_radr_ = 0.3;

    //radar measurement noise standard deviation angle in rad
    ukf.std_radphi_ = 0.0175;

    //radar measurement noise standard deviation radius change in m/s
    ukf.std_radrd_ = 0.1;

    cout << "***running UpdateRadar***"<<endl;
    ukf.UpdateRadar(meas_package.raw_measurements_);
    cout << "***Comparing Results***"<<endl;
    for(int i=0; i < 5; ++i)
    {
        if( round(expected_x(i)*1000) != round(ukf.x_(i)*1000) )
        {
            cout<<"expected_x("<<i<<") does not equal ukf.x_("
                <<i<<")"<<endl;
            cout << expected_x(i) << " != " << ukf.x_(i)<<endl;
            passed = false;
        }
    }
    return passed;
}