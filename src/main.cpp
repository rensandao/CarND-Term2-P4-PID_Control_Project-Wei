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

  PID pid, pid_throttle;
  // TODO: Initialize the pid variable.

  /*
  1. How to set the first pid parameters?　How to find the final threshold?（Hard manual tuning)

    firstly, just adjust the proportion value - kp(0.1,0.3,...), and keep ki and kd as zero. The car can move a certain distance,
    but then, because of inertia, it went out of the lane. That means a big fluctuation, and the P controller cannot hold
    it well.
    Now, set kd value from 0.3 to 3. Fluctuation came down , as the car basically moved within the lane. That's a good signal. 
    But in some place such as sharp turn, a little bigger fluctuation kept still. So we can see the car adjust itself
    more rapidly.
    once kp was set as higher than about 0.5, regulation would be be faster and more sensitive visually, but with a high  influence
    of inertia, The car can move outside the lane which means high overshoot.  So　after some experiment, the kp value was locked in [0.3，0.4], 
    while kd = 3 or 3.3 and ki=0.  

    for ki(for values > 0.01), it even showed worse influence on model performance. So ki should be even smaller.

   
  
  2. twiddle: auto-tuning or self_tuning of PID settings 

    自动调节的原理：
    调整的对象是 PID三个参数；评定方法是均方值；
    

  3. SGD

  4. combined?

  5. Using another PID to control the speed
  
  */

  double kp =  0.3; //0.33;
  double ki =  0.0023; //0.0045;
  double kd =  3.332;//3; 2.8; 2.5;2;
  pid.Init(kp,ki,kd);

  //double kp1 = 0.1;
  //double ki1 = 0.0;
  //double kd1 = 0.23;
  //pid_throttle.Init(kp1,ki1,kd1);


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')  //判断服务器是否数据传送
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s); //处理服务器数据（JSON格式），转换为javascript对象。类似字典
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          //double speed = std::stod(j[1]["speed"].get<std::string>());
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          //double throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();  

          if(steer_value > 1.0) steer_value = 1.0;
          if(steer_value < -1.0) steer_value = -1.0;

          //pid_throttle.UpdateError(speed - 20);
          //throttle_value = pid_throttle.TotalError();

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << "CTE: " << cte << " throttle value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;  //0.3
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);  //数据发给谁
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
