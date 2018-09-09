# PID-Controller 
Implement a PID controller in C++ to maneuver the vehicle around the track on simulator provided by
Udacity Self-Driving Car Engineering Nanodegree Program. 
======

## Reflection

The author changed and wrote source codes (src/main.cpp, PID.h, PID.cpp). To find PID parameters, (**Kp**, **Ki**, **Kd**), I implemented a modified Twiddle algorithm. 

### Notes on source code

  * `PID.cpp` and `PID.h`: The newly written `PID` class has a derived class called `PID_CALIBRATE`. Both `PID_CALIBRATE` and `PID` class objects compute steering angles of the car within the simulator. The drived class `PID_CALIBRATE` runs calibration algorithm and updates the parameters.
  * One can choose one of these two classes in `main.cpp` L36.
   ```
  // calibration mode
  PID_CALIBRATE pid;
  
  // driving mode
  PID pid;
   ```

### Choosing parameters 

1. To find a starting point, I let throttle be 0.1 (very small) and found a working value of **Kp**, 0.5. 
2. I let the initial **Kp**, **Ki**, **Kd** be 0.5, 0.0001, 0.0001, respectively.
3. I ran the simulator with the above initial parameters and ran the modified-Twiddle function with a throttle value of 0.2. 
  * The algorithm updates one of three parameters **Kp**, **Ki**, and **Kd** every 200 data intake, or steps. 
  * This modified Twiddle algorithm does not have a tolerance value. It keeps updating the parameters.
4. I found out that the car's steering angle *osciallated* too much, which indicated the value of **Kd** was not large enough. Hence, I let **Kd = 0.3** and gradually increased the value. In addition, I let the update happens every 100 steps instead of 200 steps, to reduce the osillation frequency while calibrating. I ran the simulator many times, monitoring and updating the parameter values.
5. Finally, I observed that **Kd** is stable around **60**. Similarly, I observed that **Kp = 1.** and **Ki = 0.01**.
------

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.


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


