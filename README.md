# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## How did I manually tune PID hyperparameters?(Reflection)
* Firstly set proportion hyperparameter(depicts with `kp` in code), tune the value(such as 0.1,0.3,1,2,3...). The basic performance is that the car can easily go out the lane at first, with the value increasing, the car can move farther but quickly lose its balance and drive out of the lane. 

    The reason for these phenomenons is, `P controller` can quickly adjust the car to move back to the center line of the lane as kp comes higher. It is easliy to understand that when kp are too small, the car move so short distance that quickly leave the normal lane and even ｒｏｔates or move backwards. But if kp is too large, ｕnder the effect of P controller, the car can also move out of the lane. Because the car have its inertia, with `ｒｅlatively high inertia` and `narrow lane range`, It is quitely hard for P controller to drag back the car. The final value for kp I set was 0.3 which is also a ａｐｐroximation.

* For now, It is time for setting `kd`(differential parameter）. Tune the value from 0.1, 0.3....to 3, 3.3, 3.5+ . When adding differential hyperparameter, obvious and positive infulence showed up. The car can run along the track without stepping out of the lane, which proves `Differential effect` can resist big fluctuation or the infulence of inertia. That's a good signal. But if kd is larger than 5.0 such as 10.0, the consequence is that the car's steer angle adjusts so frequent that it cannot drive at a higher speed than 15mph.

    With the setting `kp = 0.3, kd = 3.331, ki = 0.0`I tune, the car can drive itself for laps within the track. But in some place such as sharp turn, a little bigger fluctuation kept still. So we can see the car need adjust itself more rapidly which brought instablity.
    
    
* For `ki`(integration section), if visually, I didn't feel better promotion for fluctation. But from the real-time data, when ki = 0, the `i_error` was increasing quickly. That means totally the car tended to one side of center line for most time. So I finally set `ki = 0.0023` mainly limiting i_error to zero closely. But as I said before, visually I didn't feel
any obvious promotion. If ki is too large, its effect will affect negativally, making the car move in chaos.

Finally, PID setting is `kp = 0.3, kd = 3.331, ki = 0.0023`, while the speed limit is 30mph. Notice that this is not the only or even the best.

So for manual tuning, there is at least two points as follows:
* Manual tuning is hard but necessary, the process of tuning help us better understand every single effect of PID settings.
But as It's hard and lowly accurate, to find relatively precise value or threshold, self-tuning or auto-tuning seems necessary.
* Manual tuning above is base on the case that car's speed limit equals 30mph. Once I change the speed limit, original PID settings are not longer fit for new situation. If manually tuning again, that quitely make people crazy. 

## Twiddle or Auto-tuning(to be continued)
## PID for speed(to be continued)

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

