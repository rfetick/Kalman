# Kalman
Implement Kalman filter for your Arduino projects

Version: 1.0 (24 Aug 2019)

## Description

### Motivation

Other Kalman libraries already exist for Arduino, but so far I have only seen filters applyied to independent scalars. The matricial implementation of this project allows to use the full power of the Kalman filter to coupled variables.

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

BasicLinearAlgebra: You might find it in the available libraries in your Arduino IDE, or directly download it at https://github.com/tomstewart89/BasicLinearAlgebra

### Adding the Kalman library

Download this project and add it to you `Arduino/libraries/` folder.

You can also add all the `Kalman` files in the same folder of one of your Arduino project for unique usage (not polluting your `library/`)

## License

See the LICENSE file included

## Contact

Please send me your comments or issues in the Github dedicated `Issues` tab.
