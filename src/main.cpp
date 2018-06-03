#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

extern double steering_limit, throttle_limit;

// forward declearation
Eigen::VectorXd polyfit(Eigen::VectorXd xvals,
                        Eigen::VectorXd yvals,
                        int order);
double polyeval(Eigen::VectorXd coeffs, double x);

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

std::vector<double> transform(double offset_x, double offset_y, double rot, double x, double y)
{
  // printf("transform");
  Eigen::MatrixXd translation = MatrixXd::Identity(3, 3);
  translation(0, 2) = -offset_x;
  translation(1, 2) = -offset_y;

  // cout << "translation" << endl << translation << endl;

  Eigen::MatrixXd rotation(3, 3);
  rot = -rot;
  rotation << cos(rot), -sin(rot), 0,
              sin(rot), cos(rot), 0,
              0, 0, 1;

  // cout << "rotation" << endl << rotation << endl;
  Eigen::Vector3d v;
  v << x, y, 1;
  // cout << "in" << endl << v << endl;
  Eigen::VectorXd v_out = rotation*(translation*v);

  // cout << "out" << endl << v_out << endl;
  vector<double> output;
  output.push_back(v_out[0]);
  output.push_back(v_out[1]);

  return output;
}
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
VectorXd projection(VectorXd state, double delta, double a, double dt)
{
  VectorXd new_state(6);
  double px = state[0], py = state[1], psi = state[2],
         v = state[3], cte = state[4], epsi = state[5];

  new_state[0] = px + v * cos(psi) * dt;
  new_state[1] = py + v * sin(psi) * dt;
  new_state[2] = psi + v * delta / Lf * dt;
  new_state[3] = v + a * dt;
  new_state[4] = cte + v * sin(epsi) * dt;
  new_state[5] = epsi + v * delta / Lf * dt;

  return new_state;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];
          double delay_ms = 100.0;

          // state after delay_ms
          double delay_s = (delay_ms)/1000.0;
          double dt = 10/1000.0;
          delta = -delta*deg2rad(25);
          a *= throttle_limit;
          for (int i=0; i<int(delay_s/dt); i++) {
            px += v * cos(psi) * dt;
            py += v * sin(psi) * dt;
            psi += v * delta / Lf * dt;
            v += a * dt;
          }


          for (int it=0; it<ptsx.size(); it++)
          {
            auto pt = transform(px, py, psi, ptsx[it], ptsy[it]);
            ptsx[it] = pt[0];
            ptsy[it] = pt[1];
          }
          px = 0;
          py = 0;
          psi = 0;
          Eigen::Map<Eigen::VectorXd> x_data(ptsx.data(), ptsx.size());
          Eigen::Map<Eigen::VectorXd> y_data(ptsy.data(), ptsy.size());
          auto coeffs = polyfit(x_data, y_data, 3);

          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          double cte = polyeval(coeffs, px) - py;
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = psi - atan(coeffs[1]);

          // join all state variables into a single vector
          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          // project state into the future to account for delay in control feedback loop
          // state = projection(state, delta, a, delay/1000);

          // solve for the best actuation values
          auto t1 = std::chrono::steady_clock::now();
          auto vars = mpc.Solve(state, coeffs);
          auto t2 = std::chrono::steady_clock::now();
          auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
          std::cout << "It took me " << time_span.count() << " seconds." << endl;

          double steer_value = -vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/steering_limit;
          msgJson["throttle"] = throttle_value/throttle_limit;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // cout << "var size" << vars.size() << endl;
          for (int it=2; it<vars.size(); it+=2)
          {
            mpc_x_vals.push_back(vars[it]);
            mpc_y_vals.push_back(vars[it+1]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // for (int it=2; it<vars.size(); it+=2)
          // {
          //   auto ptx = vars[it];
          //   next_x_vals.push_back(ptx);
          //   next_y_vals.push_back(polyeval(coeffs, ptx));
          // }

          for (double i = 0; i < 50; i += 5){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(int(delay_ms)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
