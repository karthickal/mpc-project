# About

This repository contains an implementation of a Model Predictive Control in C++. This project is a part of [Udacity's Self Driving Car Engineer](https://in.udacity.com/course/self-driving-car-engineer-nanodegree--nd013/).
 
# Overview

The Model Predictive Controller, an advanced method to control processes, is used to actuate Self Driving Cars to follow a predicted trajectory. This repository contains code to move a car in a simulator (developed by Udacity). On running, the code feeds steering angles and throttle values to the simulator which then simulates the movement of the car around a track. The goal is to ensure the car follows the track and drives sensibly.  

# Solution

The following steps were implemented to meet the objective - 

1. Get the current state values from the simulator.
2. Convert the state values from the Map space to the Vehicle space.
3. Fit a polynomial to the reference trajectory in order to determine the co-efficients. 
4. Predict the state to a future point to accommodate latency in the control mechanism
5. Use the predicted state and co-efficients to solve the MPC using constraints and the kinematic motion model.
6. Feed the first state value from the solution to the simulator and repeat.

## The Model

The MPC model accepts input state, constraints and a reference trajectory. It then finds out the best state by solving for the least cost. 

Our input state consists of the following variables - 
 * px; the X co-ordinate
 * py; the Y co-ordinate
 * v; the speed
 * psi; the heading angle
 * cte; the cross track error
 * epsi; the error in the angle
 
while the actuators are made up of - 
 * delta; the steering angle
 * throttle; the acceleration component
 
The solver requires variables (state variables and actuators for a particular length), variable bounds (the maximum upper and lower value for the variables) and constraint bounds (the expected value to solve). The length is controlled by a hyperparameter called Horizon Length, which is discussed in detail in the following sections.
  
The solver finds the best fit for the variables by equating it with the kinematic motion model.

Here is the code that implements this - 

```
    MPC.cpp ln:122

    // Solve for the kinematic model
    fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
    fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
    fg[1 + cte_start + t] =
            cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
    fg[1 + epsi_start + t] =
            epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

where x0, y0, v0, psi0, cte0, epsi0, delta0 and a0 are the previous values while x1, y1, v1, psi1, cte1, epsi1, delta1 and a1 represent the current state. For example, the first line basically means *x1 = x0 + v0 * cos(psi0) * dt*.

 The solver finds the best states that gives the least cost. The cost component penalizes the differences from the expected values for the variables. This is implemented here - 
 
```
    MPC.cpp ln:67

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
        fg[0] += cf_cte * CppAD::pow(vars[cte_start + t], 2);
        fg[0] += cf_epsi * CppAD::pow(vars[epsi_start + t], 2);
        fg[0] += cf_ref_v * CppAD::pow(vars[v_start + t] - IDEAL_VELOCITY, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
        fg[0] += cf_steering * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += cf_throttle * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
        fg[0] += cf_seq_steering * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += cf_seq_throttle * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

The cost component also tries to minimizes the gap between sequential actuations.

## Space Conversion

Before the state variables are fed to the solver, they are converted from the map space to the vehicle space. The X-axis in the map space is converted to an axis which is straight ahead from the car in the vehicle space. While the Y-axis is how much the car is in the left on the vehicle space. 

Here is the implementation - 
```
    main.cpp ln:62
    
    // for each of the map space, convert to vehicle space
    for (int i = 0; i < pts_x.size(); ++i) {

        // calculate the translation and rotation for each of the points
        transformed_x = ((pts_x[i] - px) * cos(psi)) + ((pts_y[i] - py) * sin(psi));
        transformed_y = ((pts_y[i] - py) * cos(psi)) - ((pts_x[i] - px) * sin(psi));

        // store the values in the vector
        transformed_pts_x.push_back(transformed_x);
        transformed_pts_y.push_back(transformed_y);
    }
```

## Hyperparameters

The critical part of the solution depends on the value of the hyperparameters. The hyperparameters available are - 
1. Horizon Length; the number of steps ahead
2. Timestep; the time under consideration for each step

Apart from the above, hyperparameters were also defined for each of the cost component to control the importance of each part.

The final values selected for the Horizon Length is 20 for a timestep of 0.01. The observation is that a longer Horizontal Length makes the car go outside the track during sharp curves. While, a larger timestep takes a long time for the car to converge to the reference path. I started out with 10 length for a timestep of 0.5 and gradually changed these values until I got a decent solution. I then varied the multipliers for cost to create a better driving experience. The steering and the subsequent steering cost components were especially given large values to stop the car from swerving.

## Latency

Just like the real world, the simulator has a latency in sending the control values back to the vehicle. This means the solver should receive future state values in order to predict the best throttle and steering angle. The point in the car space was predicted to a point in the future (latency time) like this -
 
```
    main.cpp ln:150
    
    px = v * LATENCY / 1000;
    py = 0.0;
    psi = -((v / Lf) * delta * LATENCY / 1000);
```

px represents the position of the car straight ahead, the current position was assumed to be at 0 and the new position is then simply the distance it would have moved during the latency period.

py is still 0

while psi was determined using the current steering angle (delta).

## Sources and References

1. Equations and base code from Udacity's Self Driving Car Nanodegree (https://in.udacity.com/course/self-driving-car-engineer-nanodegree--nd013/)
2. Udacity's discussion forums

# Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

# Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.