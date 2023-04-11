/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Errors
    */
  	double p_error, i_error, d_error;

    /*
    * Coefficients
    */
  	// 在PID控制器中，kp，ki和kd是比例、积分和微分项的系数，它们用于计算控制信号。这些系数是手动调整的，以实现所需的控制性能。
	double Kp, Ki, Kd;
  
    /*
    * Output limits
    */
  	/* 
    当我们在设计PID控制器时，通常需要设置输出值的最小和最大限制，以确保控制器的输出值不会超出可接受的范围。这些最小和最大限制通常称为minLimit和maxLimit。
    例如，假设我们正在设计一个PID控制器来控制一个自动驾驶汽车的速度。我们希望控制器的输出值在-1到1之间，其中-1表示最大制动力，1表示最大加速力。在这种情况下，我们可以将minLimit设置为-1，maxLimit设置为1，以确保控制器的输出值不会超出这个范围。
    */
    double output_lim_min, output_lim_max;
  
    /*
    * Delta time
    */
  	//为微分准备的
	double delta_time;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


