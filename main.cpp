#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


bool twiddle_enabled = false;
// Added for "twiddling" the PID coefficients
double twiddle_best_error_ = 1000000;
int twiddle_state_ = 0;
int twiddle_idx = 0;
int twiddle_iterations_ = 0;
std::vector<double> p = {0, 0, 0};
std::vector<double> dp = {1, 1, 1};





void twiddle(PID &pid_control) {
  std::cout << "State: " << twiddle_state_ << std::endl;
  std::cout << "PID Error: " << pid_control.TotalError() << ", Best Error: " << twiddle_best_error_ << std::endl;

  double sum = dp[0]+dp[1]+dp[2];
  //std::cout << "SUM of DP > 0.2 : " << sum << std::endl;

  if(sum <= 0.2){
	  std::cout << "SUM of DP < 0.2 : " << sum << std::endl;
	  return;
  }



  if (twiddle_state_ == 0) {
    twiddle_best_error_ = pid_control.TotalError();
    p[twiddle_idx] += dp[twiddle_idx];
    twiddle_state_ = 1;
    std::cout << "Step 0 : "  << std::endl;
    return;

  } else if (twiddle_state_ == 1) {
    if (pid_control.TotalError() < twiddle_best_error_) {
      twiddle_best_error_ = pid_control.TotalError();
      dp[twiddle_idx] *= 1.1;
      twiddle_idx = (twiddle_idx + 1) % 3; //rotate over the 3 vector indices
      p[twiddle_idx] += dp[twiddle_idx];
      twiddle_state_ = 1;
      std::cout << "Step 1.0 : "  << std::endl;
    } else {
      p[twiddle_idx] -= 2 * dp[twiddle_idx];
      if (p[twiddle_idx] < 0) {
        p[twiddle_idx] = 0;
        twiddle_idx = (twiddle_idx + 1) % 3;
      }
      std::cout << "Step 1.1 : "  << std::endl;
      twiddle_state_ = 2;
    }
    return;

  } else { //twiddle_state_ = 2
    if (pid_control.TotalError() < twiddle_best_error_) {
      twiddle_best_error_ = pid_control.TotalError();
      dp[twiddle_idx] *= 1.1;
      twiddle_idx = (twiddle_idx + 1) % 3;
      p[twiddle_idx] += dp[twiddle_idx];
      twiddle_state_ = 1;
      std::cout << "Step 2.0 : "  << std::endl;
    } else {
      p[twiddle_idx] += dp[twiddle_idx];
      dp[twiddle_idx] *= 0.9;
      twiddle_idx = (twiddle_idx + 1) % 3;
      p[twiddle_idx] += dp[twiddle_idx];
      twiddle_state_ = 1;
      std::cout << "Step 2.1 : "  << std::endl;
      //pid.Init(p[0], p[1], p[2]);
    }
  }

  pid_control.Init(p[0], p[1], p[2]);
}


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // TODO: Initialize the pid variable.

  PID pid;
  PID speed_pid;


  std::vector<double> hyperparams;
  //Enable only P Controller
  //hyperparams = {0.15,0,0};

  //Enable the D Controller
    //hyperparams = {0.15,0.00009,0};


  //Enable the PID Controller
  hyperparams = {0.15,0.0003,3};


  pid.Init(hyperparams[0], hyperparams[1], hyperparams[2]);
  speed_pid.Init(hyperparams[0], hyperparams[1], hyperparams[2]);

  h.onMessage([&pid,&speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
        	// j[1] is the data JSON object
			  double cte = std::stod(j[1]["cte"].get<std::string>());
			  double speed = std::stod(j[1]["speed"].get<std::string>());
			  double angle = std::stod(j[1]["steering_angle"].get<std::string>());
			  double steer_value;

			  /*
			  * TODO: Calcuate steering value here, remember the steering value is
			  * [-1, 1].
			  * NOTE: Feel free to play around with the throttle and speed. Maybe use
			  * another PID controller to control the speed!
			  */
			  double speed_value;
			  double required_speed = 30.0;

			 /*
			* Calcuate steering value here, remember the steering value is
			 * [-1, 1].
			 * NOTE: Another PID controller is used to set the speed.
			 */
			 pid.UpdateError(cte);
			 steer_value = pid.TotalError();
			 if (steer_value > 1.0) {
				 steer_value = 1.0;
			 }
			 if (steer_value < -1.0) {
				 steer_value = -1.0;
			 }

			 double speed_error = speed - required_speed;
			 speed_pid.UpdateError(speed_error);
			 speed_value =  speed_pid.TotalError();

			 if (!twiddle_enabled) {
				 	 // DEBUG
				 std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed_value << std::endl;
			 }
			 else{
				 twiddle_iterations_++;
				 // Let it start running a bit first and also reset if car crashes
				 twiddle(pid);
				 std::cout << "P VECTOR: " << p[0] << "\t" << p[1] << "\t" << p[2] << std::endl;
				 std::cout << "DP VECTOR: " << dp[0] << "\t" << dp[1] << "\t" << dp[2] << std::endl;
				 twiddle_iterations_ = 0;
			 }


			 json msgJson;
			 msgJson["steering_angle"] = steer_value;
			 msgJson["throttle"] = speed_value;
			 auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			 std::cout << msg << std::endl;
			 ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
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
    return -1;
  }
  h.run();
}
