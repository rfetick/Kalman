/* 
 * This code is a minimal empty shell.
 * Fill it and modify it to match your own case.
 * 
 * Author:
 *  R.JL. Fétick
 * 
 * Revision:
 *  24 Aug 2019 - Creation
 */

#include "Kalman.h"
using namespace BLA;

//------------------------------------
/****       KALMAN PARAMETERS    ****/
//------------------------------------

#define Nstate 2
#define Nobs 2

// measurement std (to be characterized from your sensors)
#define n1 0.2
#define n2 0.1

// model std (~1/inertia). Freedom you give to relieve your evolution equation
#define m1 0.01
#define m2 0.02

KALMAN<Nstate,Nobs> K; // your Kalman filter
BLA::Matrix<Nobs> obs; // observation vector


//-----------------------------------
/****           SETUP           ****/
//-----------------------------------

void setup() {

  Serial.begin(57600);
  
  // example of evolution matrix
  K.F = {1.0, 0.0,
         0.0, 1.0};
  // example of measurement matrix
  K.H = {1.0, 0.0,
         0.0, 1.0};
  // example of measurement covariance matrix
  K.R = {n1*n1,   0.0,
           0.0, n2*n2};
  // example of model covariance matrix
  K.Q = {m1*m1,   0.0,
           0.0, m2*m2};
  
}

//-----------------------------------
/****            LOOP           ****/
//-----------------------------------

void loop() {

  // eventually update your evolution matrix inside the loop
  K.F = {1.0,  0.2,
         0.0,  1.0};
  
  // GRAB MEASUREMENT
  obs = get_sensor_data();
  
  // APPLY KALMAN FILTER
  K.update(obs);

  // PRINT RESULTS: measures and estimated state
  Serial << obs << ' ' << K.x << '\n';
}

//-----------------------------------
/****     GET SENSOR DATA       ****/
//-----------------------------------

BLA::Matrix<Nobs> get_sensor_data(){
  // It is your job to fill in this method
  // grab data from your accelerometer, GPS, etc...
}
