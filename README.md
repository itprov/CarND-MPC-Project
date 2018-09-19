# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Model Predictive controller

Model Predictive Controller (MPC) aims at finding an optimal trajectory for a vehicle. An MPC takes various actuator inputs into account, predicts trajectories, and selects a trajectory that minimizes the cost with reference to the reference trajectory. At each step, we implement actuation to get a new state using the optimized actuator inputs, then reevaluate the trajectory using the new state (i.e. constantly use receding future horizon to calculate inputs).

### The Model

The model is a kinematic model that is a simplified dynamic model, which ignores tire / internal vehicle forces, gravity, mass, inertia, air resistance, etc.
The state for this model consists of [x, y, &psi;, v] where x, y are the coordinates of the vehicle's position, &psi; is the orientation (heading direction) angle in radians, and v is the velocity.
The actuators consist of [&delta;, a] where &delta; is the steering angle, and a is the acceleration (or deceleration if negative).

The state & actuator update equations for this model are as follows:

x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> cos&psi;<sub>t</sub> dt
<br/>
y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> sin&psi;<sub>t</sub> dt
<br/>
&psi;<sub>t+1</sub> = &psi;<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>) &delta;<sub>t</sub> dt
<br/>
v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> dt

where L<sub>f</sub> is the distance between the center of mass of the vehicle and its front axle. It is set to 2.67 (given by Udacity).

There are 2 errors that we want to minimize with this model: cross-track error CTE (distance of the vehicle from reference trajectory), and orientation angle (&psi;) error e&psi; (difference of vehicle orientation and trajectory orientation). The equations for these errors are as follows:

cte<sub>t+1</sub> = cte<sub>t</sub> + v<sub>t</sub>sin(e&psi;<sub>t</sub>)dt
<br/>
i.e. cte<sub>t+1</sub> = y<sub>t</sub> - f(x<sub>t</sub>) + v<sub>t</sub>sin(e&psi;<sub>t</sub>)dt, where f(x<sub>t</sub>) is a 3<sup>rd</sup> order polynomial  c<sub>3</sub>x<sup>3</sup>+c<sub>2</sub>x<sup>2</sup>+c<sub>1</sub>x+c<sub>0</sub>.

And,
e&psi;<sub>t+1</sub> = e&psi;<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>)&delta;<sub>t</sub> dt
<br/>
i.e. e&psi;<sub>t+1</sub> = &psi;<sub>t</sub> - &psi;des<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>)&delta;<sub>t</sub> dt, where &psi;des<sub>t</sub> (desired &psi;) = arctan(f'(x<sub>t</sub>)) = arctan(3c<sub>3</sub>x<sup>2</sup>+2c<sub>2</sub>x+c<sub>1</sub>).

The model aims at finding optimum values of [&delta;, a] that minimize the cost function, which consists of 3 components:

1. Reference State component - Ensuring CTE and e&psi; are minimized, and that the vehicle doesn't stop (i.e. velocity doesn't become 0). This is achieved by adding sum of squares of CTEs, e&psi;s, and differences between v<sub>t</sub> &amp; reference velocity over all steps to the cost.

2. Actuator Controls component - Ensuring the effect of actuators [&delta;, a] is minimized (higher values are penalized), for a smooth trajectory. This is achieved by adding sum of squares of the actuator values &delta; and a to the cost. As the trajectory still resulted into minor sharp turns, I assigned weights to both the components (&Sigma; &delta;<sup>2</sup> and &Sigma; a<sup>2</sup>) and tuned them until a smoother trajectory with fairly constant speed was achieved.

3. Ensuring the effect of rate of change of actuator values, or the gap between subsequent values [&delta;<sub>t+1</sub>, a<sub>t+1</sub>] and [&delta;<sub>t</sub>, a<sub>t</sub>] is minimized (i.e. higher rate of change is penalized), for temporal smoothness. This is achieved by adding sum of squares of the differences between subsequent actuator values. As the trajectory resulted into large oscillations, I assigned weights to both the components (&Sigma; (&delta;<sub>t+1</sub> - &delta;<sub>t</sub>)<sup>2</sup> and &Sigma; (a<sub>t+1</sub> - a<sub>t</sub>)<sup>2</sup>) for smoother steering transitions and tuned those weights until a smooth trajectory with fairly constant speed was achieved. It was necessary to assign a relatively large weight to the rate of change of &delta;, and the weight value needs to be higher if the reference speed is set to a higher value.

### Timestep Length and Elapsed Duration (N & dt)

The number of timesteps is N and the time elapsed between subsequent actuations is dt. The product of these 2 numbers is the total horizon used for trajectory prediction. As latency is 100 ms, it is desirable to use a factor of 100ms as the dt value, to make it easier to deal with latency. I started with N = 10 and dt = 0.1 (100 ms). However, that resulted in the vehicle running off the track quickly, as the horizon was too large and the discretization error was also large. Therefore, I reduced the value of dt to 0.05 and increased N to 20. This reduced the discretization, but still the horizon was too large, so at sharp turns on the track, the error was too high (the vehicle veered off the track few times). I further reduced the value multiple times until the error was low enough to keep the vehicle inside the track. The final value of N was set to 15 and dt to 0.05.  

### Polynomial Fitting and MPC Preprocessing

A 3<sup>rd</sup> order polynomial is fitted to the waypoints. To make the input state vector calculation easier, the waypoints are converted from map coordinates to the vehicle's coordinates before fitting using the following formulae:

waypoint_x = x cos(-&psi;) - y sin(-&psi;)<br/>
waypoint_y = x sin(-&psi;) + y cos(-&psi;)

(It is necessary to flip &psi; to -&psi; as the simulator uses positive orientation values for clockwise rotation / right turns.)

This results in transforming x, y coordinates, and &psi; of each input to 0. It also simplifies CTE and e&psi; calculation.

### Model Predictive Control with Latency

Actuation command delay (latency) of 100 ms is factored into the model. Since the value of latency is a multiple (say M) of the chosen value of dt (i.e. latency = M dt), the actuator values &delta;<sub>t</sub>, a<sub>t</sub> used in the state update equations are simply the actuator values as of t - 1 - M<sup>th</sup> step, starting at t = M. With the chosen value of dt = 0.05, the value of M is 2, so starting at t = 2, the model uses actuator values as of t - 3 in the state update equations. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
