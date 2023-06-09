# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, you will apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

### Installation

Run the following commands to install the starter code in the Udacity Workspace:

Clone the <a href="https://github.com/udacity/nd013-c6-control-starter/tree/master" target="_blank">repository</a>:

`git clone https://github.com/udacity/nd013-c6-control-starter.git`

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles your c++ code, run it after every change in your code)

## Testing

To test your installation run the following commands.

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`


## Project Instructions

In the previous project you built a path planner for the autonomous vehicle. Now you will build the steer and throttle controller so that the car follows the trajectory.

You will design and run the a PID controller as described in the previous course.

In the directory [/pid_controller](https://github.com/udacity/nd013-c6-control-starter/tree/master/project/pid_controller)  you will find the files [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.cpp)  and [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.h). This is where you will code your pid controller.
The function pid is called in [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp).

### Step 1: Build the PID controller object
Complete the TODO in the [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.h) and [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/pid_controller.cpp).

Run the simulator and see in the desktop mode the car in the CARLA simulator. Take a screenshot and add it to your report. The car should not move in the simulation.
### Step 2: PID controller for throttle:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp), complete the TODO (step 2) to compute the error for the throttle pid. The error is the speed difference between the actual speed and the desired speed.

Useful variables:
- The last point of **v_points** vector contains the velocity computed by the path planner.
- **velocity** contains the actual velocity.
- The output of the controller should be inside [-1, 1].

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 3: PID controller for steer:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/blob/master/project/pid_controller/main.cpp), complete the TODO (step 3) to compute the error for the steer pid. The error is the angle difference between the actual steer and the desired steer to reach the planned position.

Useful variables:
- The variable **y_points** and **x_point** gives the desired trajectory planned by the path_planner.
- **yaw** gives the actual rotational angle of the car.
- The output of the controller should be inside [-1.2, 1.2].
- If needed, the position of the car is stored in the variables **x_position**, **y_position** and **z_position**

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 4: Evaluate the PID efficiency
The values of the error and the pid command are saved in thottle_data.txt and steer_data.txt.
Plot the saved values using the command (in nd013-c6-control-refresh/project):

```
python3 plot_pid.py
```

You might need to install a few additional python modules: 

```
pip3 install pandas
pip3 install matplotlib
```
### Simulator Screenshot
<img src="img/simulator.jpg"/>

### Answer the following questions:
#### Add the plots to your report and explain them (describe what you see)
Steering Plot<br>
<img src="img/steer.png"/>

Throttle Plot<br>
<img src="img/throttle.png"/>
After changing the PID value several times, the vehicle finally works without collision.
The figures above show the error & output for steering and Throttle. 
In steering plot, the steer plot correlates with the steering errors. In order to turns smoothly, I used a smaller value of KP. 
In the throttle plot, the throttle error doesn't produce large vibrations at the beginning, so the car started smoothly. The car also passed the 3 vehicles without crash.

#### What is the effect of the PID according to the plots, how each part of the PID affects the control command?
 
P: Proportional term
- "P" is proportional to the current error value. 
- For example, "P" can be used to adjust the steering angle of the car. If the car is shifting to the right, "P" will produce an value to make the car to the left.

I: Integral term
- "I" sums up the error values over time.
- Commonly, "I" corrects the car speed fluctuations over time. 
- For example, the speed is below 60km/h, "I" adds a corrective value, increasing the car's speed gradually until it reaches 60km/h. In contrast, if the speed is above 60km/h, "I" minus a corrective value, decreasing the speed to 60km/h.

D: Derivative term
- "D" is proportional to the change rate of the error value.
- "D" is used to predict the future error based on its current change rate. 
- For example, If the car's speed slows down due to climbing mountains, "D" will detect the rate of change of the error and adjust the power.

####  How would you design a way to automatically tune the PID parameters?

After doing some research, I found there are several methods to tune the PID parameters, such as genetic algorithm method, Ziegler-Nichols method, gradient descent method, etc.
In my opinion, I prefer to use the twiddle algorithm for Parameter Optimization.
1. Initialize the parameters to specific values.
2. change initial value.
3. Evaluate the system's performance using the current parameters.
4. If the performance has improved, keep the new parameter values and increase the change value for that parameter.
5. If the performance has not improved, revert the parameter value and decrease the change value for that parameter.
6. Repeat the steps above.

#### PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
Pros:<br>
1. The algorithm is really simple, its parameters are easy to modify.
2. The algorithm can work very fast, it is suitable for realtime situations.

Cons:<br>
1. PID is not very robust, it can be affected by any kind of noises.
2. PID doesn't use any models of the vehicle, so it can't predict the future state and calculate best outputs like MPC(model predictive control). 