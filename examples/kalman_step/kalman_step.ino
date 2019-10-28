/*
 * Run an example of Kalman filter.
 * This example simulates an oscillating step-like state.
 * Noisy measurements are simulated with the 'SIMULATOR_' functions. 
 * Play around with 'm1' and 'm2' gains.
 * Results are printed on Serial port. You can use 'kalman_step.py' to analyse them with Python.
 * 
 * Author:
 *  R.JL. Fétick
 *  
 * Revision:
 *  24 Aug 2019 - Creation
 * 
 */

#include <Kalman.h>
using namespace BLA;

//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 2
#define Nobs 2

// measurement std
#define n1 0.2
#define n2 0.2
// model std (1/inertia)
#define m1 0.01
#define m2 0.05


BLA::Matrix<Nobs> obs; // observation vector
KALMAN<Nstate,Nobs> K; // your Kalman filter

//------------------------------------
/****    SIMULATOR PARAMETERS   ****/
//------------------------------------

BLA::Matrix<Nstate> state; // true state vector for simulation
BLA::Matrix<Nstate> state_var; // state variations from the model
BLA::Matrix<Nobs> noise; // additive noise for simulation

#define LOOP_DELAY 10 // add delay in the simulation
#define SIMUL_FREQ 250 // oscillating frequency (in units of steps)
int SIMUL_STEP = 0; // step counter

//------------------------------------
/****        SETUP & LOOP       ****/
//------------------------------------

void setup() {

  Serial.begin(57600);

  // The model below is very simple since matrices are diagonal!
  
  // time evolution matrix
  K.F = {1.0, 0.0,
         0.0, 1.0};
  // measurement matrix
  K.H = {1.0, 0.0,
         0.0, 1.0};
  // measurement covariance matrix
  K.R = {n1*n1,   0.0,
           0.0, n2*n2};
  // model covariance matrix
  K.Q = {m1*m1,   0.0,
           0.0, m2*m2};
  
  // INITIALIZE SIMULATION
  SIMULATOR_INIT();
  
}

void loop() {

  // SIMULATE NOISY MEASUREMENT
  SIMULATOR_UPDATE();
  
  // APPLY KALMAN FILTER
  K.update(obs);

  // PRINT RESULTS: true state, measures, estimated state
  Serial << state << ' ' << obs << ' ' << K.x << '\n';
}

//------------------------------------
/****     SIMULATOR FUNCTIONS   ****/
//------------------------------------

void SIMULATOR_INIT(){
  randomSeed(analogRead(0));
  state(0) = 0.0;
  state(1) = 0.0;
  state_var(0) = 0.0;
  state_var(1) = 0.0;
}

void SIMULATOR_UPDATE(){
  // eventually update state variations with your own function
  // (e.g. system unknown accelerations)
  // strong variations will be smoothed by Kalman filter
  // due to the balance between noise filtering and reactivity
  state_var(0) = 0.0; 
  state_var(1) = 0.0;
  // Step-like function
  if(SIMUL_STEP==0){
	state_var(0) = 1.0; 
	state_var(1) = 1.0;
  }
  if(SIMUL_STEP==SIMUL_FREQ/2){
	state_var(0) = -1.0; 
	state_var(1) = -1.0;
  }
  
  state = K.F * state + state_var; // time evolution
  noise(0) = n1 * SIMULATOR_GAUSS_NOISE();
  noise(1) = n2 * SIMULATOR_GAUSS_NOISE();
  obs = K.H * state + noise; // measure
  
  SIMUL_STEP += 1;
  SIMUL_STEP = SIMUL_STEP % SIMUL_FREQ;
  
  delay(LOOP_DELAY); //add a delay in loop
}

double SIMULATOR_GAUSS_NOISE(){
  // Generate centered reduced Gaussian random number with Box-Muller algorithm
  double u1 = random(1,10000)/10000.0;
  double u2 = random(1,10000)/10000.0;
  return sqrt(-2*log(u1))*cos(2*M_PI*u2);
}
