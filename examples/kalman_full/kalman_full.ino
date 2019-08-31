/*
 * Run an example of Kalman filter.
 * This example simulates a sinusoidal position.
 * Noisy measurements are simulated with the 'SIMULATOR_' functions. 
 * Results are printed on Serial port. You can use 'kalman_full.py' to analyse them with Python.
 * 
 * Author:
 *  R.JL. Fétick
 *  
 * Revision:
 *  31 Aug 2019 - Creation
 * 
 */

#include "Kalman.h"
using namespace BLA;

//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 3 // position, speed, acceleration
#define Nobs 2   // position, acceleration

// measurement std
#define n_p 0.3
#define n_a 5.0
// model std (1/inertia)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8


BLA::Matrix<Nobs> obs; // observation vector
KALMAN<Nstate,Nobs> K; // your Kalman filter

//------------------------------------
/****    SIMULATOR PARAMETERS   ****/
//------------------------------------

unsigned long T;
float DT;

BLA::Matrix<Nstate> state; // true state vector for simulation
BLA::Matrix<Nobs> noise;   // additive noise for simulation

#define LOOP_DELAY 10  // add delay in the measurement loop
#define SIMUL_PERIOD 0.3 // oscillating period [s]
#define SIMUL_AMP 1.0

//------------------------------------
/****        SETUP & LOOP       ****/
//------------------------------------

void setup() {

  Serial.begin(57600);

  // The model below is very simple since matrices are diagonal!
  
  // time evolution matrix
  K.F = {1.0, 0.0, 0.0,
		 0.0, 1.0, 0.0,
         0.0, 0.0, 1.0};

  // measurement matrix
  K.H = {1.0, 0.0, 0.0,
         0.0, 0.0, 1.0};
  // measurement covariance matrix
  K.R = {n_p*n_p,   0.0,
           0.0, n_a*n_a};
  // model covariance matrix
  K.Q = {m_p*m_p,     0.0,     0.0,
             0.0, m_s*m_s,     0.0,
			 0.0,     0.0, m_a*m_a};
  
  T = millis();
  
  // INITIALIZE SIMULATION
  SIMULATOR_INIT();
  
}

void loop() {
	
  // TIME COMPUTATION
  DT = (millis()-T)/1000.0;
  T = millis();

  K.F = {1.0,  DT,  DT*DT/2,
		 0.0, 1.0,       DT,
         0.0, 0.0,      1.0};

  // SIMULATE NOISY MEASUREMENT
  SIMULATOR_UPDATE();
  
  // APPLY KALMAN FILTER
  K.update(obs);

  // PRINT RESULTS: true state, measures, estimated state
  Serial << state << ' ' << obs << ' ' << K.x << ' ' << K.P << '\n';
}

//------------------------------------
/****     SIMULATOR FUNCTIONS   ****/
//------------------------------------

void SIMULATOR_INIT(){
  randomSeed(analogRead(0));
  state.Fill(0.0);
  obs.Fill(0.0);
}

void SIMULATOR_UPDATE(){
	unsigned long tcur = millis();
  state(0) = SIMUL_AMP*sin(tcur/1000.0/SIMUL_PERIOD);
  state(1) = SIMUL_AMP/SIMUL_PERIOD*cos(tcur/1000.0/SIMUL_PERIOD);
  state(2) = -SIMUL_AMP/SIMUL_PERIOD/SIMUL_PERIOD*sin(tcur/1000.0/SIMUL_PERIOD);
  
  noise(0) = n_p * SIMULATOR_GAUSS_NOISE();
  noise(1) = n_a * SIMULATOR_GAUSS_NOISE();
  obs = K.H * state + noise; // measure
  
  delay(LOOP_DELAY); //add a delay in measurement
}

double SIMULATOR_GAUSS_NOISE(){
  // Generate centered reduced Gaussian random number with Box-Muller algorithm
  double u1 = random(1,10000)/10000.0;
  double u2 = random(1,10000)/10000.0;
  return sqrt(-2*log(u1))*cos(2*M_PI*u2);
}