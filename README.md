# MPC
Model Predictive Controller to keep a car on track

---

## Dependencies

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


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## The Model

The model is basically the same as shown in the [project instructions](http://bsft.io/x/8uxfcm?uid=eb0ca4a3-b31f-4b1e-9a58-7b9f5289dcea&mid=80b300a4-ca7b-4144-9be1-a8ee2170650d). The state is given by position (x,y), speed (v), orientation (psi) and the errors: cross track error (cte) and orientation error (epsi). The actuators are given by steering angle (delta) and acceleration (a). The update quations for the state are given by

1. x(t+dt) = x(t) + v(t) * cos(psi(t)) * dt,
2. y(t+dt) = y(t) + v(t) * sin(psi(t)) * dt,
3. v(t+dt) = v(t) + a(t) * dt,
4. psi(t+dt) = psi(t) + v(t) * delta(t) / Lf * dt,
5. cte(t+dt) = cte(t) + v(t) * sin(epsi(t)) * dt,
6. epsi(t+dt) = epsi(t) - v(t) * delta(t) / Lf * dt.

## Number of Time Steps & Timestep Length

The time step length dt is taken from the [project instructions](http://bsft.io/x/8uxfcm?uid=eb0ca4a3-b31f-4b1e-9a58-7b9f5289dcea&mid=80b300a4-ca7b-4144-9be1-a8ee2170650d), dt=0.1, while the number of time steps has been set to N = 1.5 / dt = 15, corresponding to 1.5 seconds. 1.5s turned out to be sufficient for stable control (1s was not); reducing dt while keeping N*dt constant led to a huge increase in computation effort. 

## Latency

The simulation implements a delay of 100ms for the computation of new actuator values in order to simulate the dynamics of the actuators. This can be take into account by extending the model introducing state variables for the actuators. In this solution, I do so using state variables s_delta and s_a for the corresponding actuators delta and a. The update equations for this are implemented as

7. s_delta(t+dt) = (1 - filter) * s_delta(t) + filter * delta(t),
8. s_a(t+dt) = (1 - filter) * s_a(t) + filter * a(t).

The variable filter is a number between 0 and 1. 0 means, that the state of the actuator does not react on the actuation, while 1 leads to an immediate reaction. In order to reflect the delay in the update equations, one has to replace delta(t) by s_delta(t+dt) and a(t) by s_a(t+dt) in equations 4. and 6, while s_delta(t) and s_a(t) are the state of the actuators from the last time step.



