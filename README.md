# Kalman ![bdg](https://img.shields.io/github/license/rfetick/Kalman) ![bdg](https://img.shields.io/github/v/tag/rfetick/Kalman) ![bdg](https://img.shields.io/github/v/release/rfetick/Kalman)
**Implement Kalman filter for your Arduino projects**

:information_source: See the [VERSION](VERSION.md) and the [LICENSE](LICENSE) files

:ballot_box_with_check: Tested successfully on _Arduino Uno_ and _Nano_ (ATmega 328P old bootloader) 

:arrows_counterclockwise: Any issue or successful test? Your feedback is important for improving this library. See the _Contact_ section at the end of this file or write to the [Group](https://groups.google.com/forum/#!forum/kalman-for-arduino)!

## Description

Other Kalman libraries already exist for Arduino, but so far I have only seen filters applied to independent scalars. The matricial implementation of this project allows to use the full power of the Kalman filter to coupled variables. It allows to merge measurements from multiple sensors such as accelerometers, GPS, ultrasound (distance) or pressure (altitude) sensors...

This library is adapted to your most sophisticated projects. In order to use it you need some knowledge about matrix formalism and be able to write (or find on internet) the actual state equations of your system.

## Installation

### Prerequisites

`BasicLinearAlgebra`: You might find it in the library manager of your Arduino IDE, or directly download it at https://github.com/tomstewart89/BasicLinearAlgebra

### Adding the Kalman library

This library is available in the official Arduino library manager. Just type `Kalman` and you should find it.

Other possibility is to download (or clone) this project and add it to your `Arduino/libraries/` folder.

## Start using the library in your Arduino projects

:information_source: See the [GETTING_STARTED](GETTING_STARTED.md) file

## Possible issues

* The library `BLA::Matrix` seems to throw errors for matrices of size `<1,1>`. So the Kalman library will only work for `Nstate>1` and `Nobs>1`. For one-dimensional Kalman filters, please refer to other Arduino libraries.

* Size of matrices has to be relatively small due to the limited SRAM memory of Arduino. Effort has been made to reduce SRAM usage.

* An issue has been reported with Arduino Nano IoT 33: the program compiles but is not correctly loaded to Arduino Nano IoT 33. I am investigating this.

## Contact

Issues must be reported in the Github dedicated `Issues` tab.

This library is a work in progress, and your feedback is welcome at [this link](https://groups.google.com/forum/#!forum/kalman-for-arduino). I am looking for advices or contributors to improve the project.

Please also send me comments on any successful test performed with your Arduino board. A feedback on the library is important for me an for other users!!!
