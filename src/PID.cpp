#include "PID.h"
#include <limits>
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {

Kp = kp;
Ki = ki;
Kd = kd;

p_error = 0.0;
//p_error = numeric_limits<double>::max();
i_error = 0.0;
d_error = 0.0;


}

void PID::UpdateError(double cte) {
    
  //  if(p_error == numeric_limits<double>::max()){
   //     p_error = cte;
   // }

    d_error = cte - p_error;
    p_error =  cte;
    i_error += p_error;
    
      
    std::cout<< "--------" <<endl;
    std::cout<< "p_error: "<< p_error << " i_error: "<< i_error <<" d_error: "<<d_error << endl;

}


double PID::TotalError() {
    double total_error;
    total_error = -(Kp * p_error + Ki * i_error + Kd * d_error);

    return total_error;
}

