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
  /*
  ------ Step1 ------
  TODO: Initialize PID coefficients (and errors, if needed)
  */
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
  
  delta_time = 0.0;//微分用
}

void PID::UpdateError(double cte) {
  /*
  ------ Step2 ------
  TODO: Update PID errors based on cte.
  作用: 根据当前误差来更新这三个误差变量。这些误差变量会在后续的控制中使用，从而帮助系统更好地控制输出。
  参数:
  	cte表示当前的横向偏差，即车辆实际位置与期望轨迹之间的差异。
  	p_error、i_error和d_error变量分别表示PID控制器的比例、积分和微分项误差。
  */
  
  if(delta_time>0){
    p_error = cte;
    
  	//积分项的误差需要考虑历史误差的累加. 因此，需要使用+号将当前误差值累加到历史误差值中
    i_error = i_error + cte*delta_time;

    //微分项的误差需要考虑当前误差与上一次误差的差异。因此,需要使用-号计算当前误差与上一次误差的差异
    d_error = (cte-p_error) / delta_time;
  
  }else{
    d_error = 0.0;
  } 
}

double PID::TotalError() {
  /*
  ------ Step3 ------
  TODO: Calculate and return the total error
  The code should return a value in the interval [output_lim_mini, output_lim_maxi]  
  作用:根据这些误差值计算控制输出，即对车辆的转向角进行校正。
  	  它将比例、积分和微分项误差乘以相应的系数，并将它们相加。
  	  最终的控制输出是这个总和的相反数，因为我们希望将控制输出应用于车辆的转向角，而转向角的变化方向与控制输出的变化方向相反。
  -符号是用来将控制输出取反的。
  	我们希望将控制输出应用于车辆的转向角，而转向角的变化方向与控制输出的变化方向相反。  
  	因此，我们需要将控制输出取反，以便正确地将其应用于车辆的转向角。
  */
  double control =  Kp*p_error + Ki*i_error + Kd*d_error;
  
  if (control < output_lim_min) {
    control = output_lim_min;
  }
  else {
    control = output_lim_max;
  }   

  return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /*
  ------ Step4 ------
  TODO: Update the delta time with new value
  PID控制器是一种反馈控制循环，用于控制系统的输出，以便与期望的输入保持一致。
  它通过比例、积分和微分项对误差进行校正，以使系统的输出尽可能接近期望的输入。
  其中，微分项是根据误差的变化率进行计算的，因此需要知道两次测量之间的时间差，即delta_time。
  */
  delta_time = new_delta_time;
  return delta_time;
}