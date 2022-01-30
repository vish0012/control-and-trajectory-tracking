/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
  cte_curr = 0.0;
  prev_cte = 0.0;
  int_cte = 0.0;
  diff_cte = 0.0;
  delta_time = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  cte_curr = cte;
  diff_cte = (cte_curr - prev_cte) / delta_time;
  int_cte += cte_curr * delta_time;
  prev_cte = cte;
  
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
    control = - Kp * cte_curr - Ki * int_cte - Kd * diff_cte;
  	control = max(min(control, output_lim_max), output_lim_min);
    std::cout << "new control : " << control << std::endl;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
  std::cout << "new delta time: " << delta_time << std::endl;
  return delta_time;
}