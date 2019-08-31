# Kalman ![bdg](https://img.shields.io/github/license/rfetick/Kalman) ![bdg](https://img.shields.io/github/v/tag/rfetick/Kalman) ![bdg](https://img.shields.io/github/v/release/rfetick/Kalman)
**Implement Kalman filter for your Arduino projects**

:information_source: See the [VERSION](VERSION.md) file

:ballot_box_with_check: See the [LICENSE](LICENSE) file

:arrows_counterclockwise: Your feedback is important for improving this library. See the _Contact_ section at the end of this file

## Description

### Motivation

Other Kalman libraries already exist for Arduino, but so far I have only seen filters applied to independent scalars. The matricial implementation of this project allows to use the full power of the Kalman filter to coupled variables. It allows to merge measurements from multiple sensors such as accelerometers, GPS, ultrasound (distance) or pressure (altitude) sensors...

This library is adapted to your most sophisticated projects. In order to use it you need some knowledge about matrix formalism and be able to write (or find on internet) the actual state equations of your system.

### Equations

Your state evolution model and state observation are respectively given by the matrix equations

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;x<sub>k</sub> = F<sub>k</sub> x<sub>k-1</sub> + B<sub>k</sub> u<sub>k</sub> + q<sub>k</sub>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;y<sub>k</sub> = H<sub>k</sub> x<sub>k</sub> + r<sub>k</sub>

where _k_ is the time step, _x_ the state vector, and _y_ the measurement vector. The full list of definitions and respective dimensions are summarized in the table below.

| NAME | DIMENSION       | DEFINITION                       |
|------|-----------------|----------------------------------|
| x    | Nstate          | State vector                     |
| F    | Nstate x Nstate | Time evolution matrix            |
| B    | Nstate x Ncom   | Command matrix (optional)        |
| u    | Ncom            | Command vector                   |
| Q    | Nstate x Nstate | Model covariance (~1/inertia)    |
| y    | Nobs            | Observation vector (measurement) |
| H    | Nobs x State    | Observation matrix               |
| R    | Nobs x Nobs     | Noise covariance                 |

## Installing

### Prerequisites

`BasicLinearAlgebra`: You might find it in the library manager of your Arduino IDE, or directly download it at https://github.com/tomstewart89/BasicLinearAlgebra

### Adding the Kalman library

Download this project and add it to your `Arduino/libraries/` folder.

## Start using the library in your Arduino projects

### Examples

Examples are provided in the so-called `examples/` subfolder. Open them to get an idea on how to use the Kalman library.

* `kalman_minimal` : Empty shell ready to be filled with your state equations and measurements

* `kalman_step` : Numerical simulation of a noisy measurement on a step function so you do not need to plug any components to play with the Kalman library. Also includes a Python file to grab the measurements from Arduino and plot them. See figure below.

![alt text](examples/kalman_step/kalman_step.png "Kalman filter applied to noisy data")

_The figure above shows two examples of Kalman filter (blue) applied on noisy data (green). The true state (red) is a step function. On the left graph, components of_ Q  _are chosen small (high inertia). Noise is efficiently filtered but response time is longer with respect to the right graph._

### Detailed explanations

Using the Kalman library in your Arduino files is (I hope) straightforward. First include the library in your `.ino` file. Also use the `BLA` (BasicLinearAlgebra) namespace since you will need to define some BLA vectors
```cpp
#include "Kalman.h"
using namespace BLA
```

Then define your number of states, number of observations and eventually number of commands. For the sake of simplicity, we will start without any command
```cpp
#define Nstate 2
#define Nobs 2
```

You can now define a Kalman filter and an observation vector
```cpp
KALMAN<Nstate, Nobs> K;
BLA::Matrix<Nobs> obs;
```

In the `setup` and `loop` functions you can now access to all the matrices `K.F`, `K.H`, `K.x`, `K.Q`, `K.R`, `K.P`... You can modify them, but be careful that inloop modifications of `K.x` or `K.P` might lead to unconsistent results! If you want to access to the `K.x` estimate, it is better to use the method
```cpp
BLA::Matrix<Nstate> my_x = K.getxcopy();
```

And of course you can update your Kalman filter with a new measure
```cpp
obs = fill_with_sensor_measures(); // grab here your sensor data and fill in the obs vector
K.update(obs);
```

## Possible issues

* The library `BLA::Matrix` seems to throw errors for matrices of size `<1,1>`. So the Kalman library will only work for `Nstate>1` and `Nobs>1`. For one-dimensional Kalman filters, please refer to other Arduino libraries.

* Size of matrices has to be small due to the limited SRAM memory of Arduino. This is the main limitation.

## Contact

Please send me your comments or issues in the Github dedicated `Issues` tab.

This library is a work in progress, and your feedback is welcome at [this link](https://groups.google.com/forum/#!forum/kalman-for-arduino). I am looking for advices or contributors to improve the project.
