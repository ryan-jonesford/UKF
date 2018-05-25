#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "Inc/ekf.h"
#include "Inc/ukf.h"
#include "Inc/tools.h"
#include "Inc/tests.h"
#include <fstream>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

KalmanFilter *kf_type(char * kf_flag){
  if( !strcmp( kf_flag,"-e" ) )
    {
      std::cout<<"Running EKF\n"<<endl;
      return new EKF();
    }
  //defalt to UKF    
  else //if( !strcmp( kf_flag, "-u" ) )
  {
    std::cout<<"Running UKF\n"<<endl;
    return new UKF();
  }
}

void run_test_UpdateRadar( KalmanFilter * KF ){
  Tests test;
  if(test.test_UpdateRadar()){
    std::cout<<"UpdateRadar Passed\n"<<endl;
  } else{
    std::cout<<"UpdateRadar Failed\n"<<endl;          
  }
}

void run_test_predictUKF( KalmanFilter * KF ){
  Tests test;
  if(test.test_PredictUKF()){
    std::cout<<"PredictUKF Passed\n"<<endl;
  } else{
    std::cout<<"PredictUKF Failed\n"<<endl;          
  }
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  // Create an output file
  bool logging = false;
  bool write_header = false;
  bool be_verbose = false;
  bool laser_only = false;
  bool radar_only = false;
  bool print_nis = false;
  KalmanFilter *KF;
  
  if ( argc > 1 )
  {
      // Create a Kalman Filter instance
    for(int inx = 1; inx < argc; ++inx)
    {
      if( !strcmp(argv[inx],"-f"))
      {
        logging = true;
        write_header=true;
      }
      else{
        KF = kf_type( argv[inx] );
      }
      if( !strcmp(argv[inx],"-tu"))
      {
        run_test_predictUKF(KF);
        run_test_UpdateRadar(KF);
        if( KF != NULL ){
          delete KF;
        }
        return 0;
      }
      if( !strcmp(argv[inx],"-v"))
      {
        be_verbose = true;
      }
      if( !strcmp(argv[inx],"-v"))
      {
        be_verbose = true;
      }
      if( !strcmp(argv[inx],"-r"))
      {
        radar_only = true;
      }if( !strcmp(argv[inx],"-l"))
      {
        laser_only = true;
      }
      if( !strcmp(argv[inx], "-n")){
        print_nis = true;
      }
    }
  }
  else
  {
    KF = new UKF();
  }
  KF->verbose_ = be_verbose;
  KF->use_laser_ = laser_only;
  KF->use_radar_ = radar_only;
  KF->print_nis_ = print_nis;



  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  h.onMessage([KF,&tools,&estimations,&ground_truth,&logging,&write_header](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    fstream fs;

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if ( s != "" ) 
      {

        if( logging )
        {
          if( write_header )
          {
            fs.open("out_log.csv", fstream::in | fstream::app);
            fs << "timestamp,px,py,rho,phi,rho_dot,est_px,est_py,est_vx,est_vy,gt_px,"
                  "gt_py,gt_vx,gt_vy,rsme(px),rsme(py),rsme(vx),rsme(vy)\n";
            write_header = false;
          }
          else
          {
            fs.open("out_log.csv", fstream::in | fstream::app);
          }
        }
      	
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;
          if (sensor_type.compare("L") == 0) 
          {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            if( logging )
            {  
              fs << timestamp<<","<<px<<","<<py<<",,,,";
            }
          } 
          else if (sensor_type.compare("R") == 0) 
          {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro,theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            if(logging)
            {
              fs <<timestamp<<","<< ",,"<<ro<<","<<theta<<","<<ro_dot<<",";
            }
          }
          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt; 
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          ground_truth.push_back(gt_values);
          
            
          //Call ProcessMeasurment(meas_package) for Kalman filter
          KF->ProcessMeasurement(meas_package);    	  

          //Push the current estimated x,y positon from the Kalman filter's state vector

          VectorXd estimate(4);
          double p_x, p_y, v, yaw, v1, v2;

          if ( !strcmp(KF->get_kf_type(), "EKF") )
          {
            p_x = KF->x_(0);
            p_y = KF->x_(1);
            v1  = KF->x_(2);
            v2  = KF->x_(3);

          }
          else
          {
            p_x = KF->x_(0);
            p_y = KF->x_(1);
            v   = KF->x_(2);
            yaw = KF->x_(3);

            v1 = cos(yaw)*v;
            v2 = sin(yaw)*v;
          }
          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;

          estimations.push_back(estimate);

          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
          if( logging )
          {
            fs  <<p_x<<","<<p_y<<","<<v1<<","<<v2<<","
                <<x_gt<<","<<y_gt<<","<<vx_gt<<","<<vy_gt<<","
                <<RMSE(0)<<","<<RMSE(1)<<","<<RMSE(2)<<","<<RMSE(3)<<"\n";
            fs.close();
          } 
        }
      } 
      else 
      {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    delete KF;
    return -1;
  }
  h.run();
}