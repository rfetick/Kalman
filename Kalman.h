/*
 * Implements the Kalman filter corresponding to the linear problem
 *    x_k = F*x_{k-1} + B*u_k + q_k   (evolution model)
 *    y_k = H*x_k + r_k               (measure)
 * 
 * with the matrices and vectors
 *    x [output] [size=Nstate]          Estimated state vector
 *    F [input]  [size=(Nstate,Nstate)] Free evolution of the state vector
 *    B [input]  [size=(Nstate,Ncom)]   [optional] Command vector acting on state
 *    Q [input]  [size=(Nstate,Nstate)] Model covariance acting as (1/inertia)
 *    y [input]  [size=Nobs]            Observed (measured) data from sensors
 *    H [input]  [size=(Nobs,Nstate)]   Observation matrix
 *    R [input]  [size=(Nobs,Nobs)]     Measurement noise covariance matrix
 * 
 * Many attributes are public, so you might modify them as you wish.
 * However be careful since modification of attributes (especially 'P' and 'x')
 * might lead to unconsistant results.
 * Use the 'getxcopy' method to get a copy of the 'x' state vector.
 * 
 * Requires:
 *  BasicLinearAlgebra  https://github.com/tomstewart89/BasicLinearAlgebra
 * 
 * License:
 *  See the LICENSE file
 * 
 * Author:
 *  R.JL. Fétick
 * 
 * Revision:
 *  23 Aug 2019 - Creation
 * 
 */

#ifndef Kalman_h
#define Kalman_h

#include <BasicLinearAlgebra.h>

#include "Arduino.h"

using namespace BLA;

template<int dim, class ElemT> struct Diagonal
{
    mutable ElemT m[dim];

    // The only requirement on this class is that it implement the () operator like so:
    typedef ElemT elem_t;

    ElemT &operator()(int row, int col) const
    {
        static ElemT dummy;

        // If it's on the diagonal and it's not larger than the matrix dimensions then return the element
        if(row == col && row < dim)
            return m[row];
        else
            // Otherwise return a zero
            return (dummy = 0);
    }
};



/**********      CLASS DEFINITION      **********/

template<int Nstate, int Nobs, int Ncom = 0>
class KALMAN{
  private:
    BLA::Matrix<Nstate,1> NULLCOMSTATE;
    void _update(BLA::Matrix<Nobs> obs, BLA::Matrix<Nstate> comstate);
    BLA::Matrix<Nstate,Nstate, Diagonal<Nstate,float> > Id; // Identity matrix
  public:
    BLA::Matrix<Nstate,Nstate> F; // time evolution matrix
    BLA::Matrix<Nobs,Nstate> H; // observation matrix
    BLA::Matrix<Nstate,Ncom> B; // Command matrix (optional)
    BLA::Matrix<Nstate,Nstate> Q; // model noise covariance matrix
    BLA::Matrix<Nobs,Nobs> R; // measure noise covariance matrix
    BLA::Matrix<Nstate,Nstate> P; // (do not modify, except to init!)
    BLA::Matrix<Nstate,1> x; // state vector (do not modify, except to init!)
    BLA::Matrix<Nobs,1> y; // innovation
    BLA::Matrix<Nobs,Nobs> S;
    BLA::Matrix<Nstate,Nobs> K; // Kalman gain matrix
    int status; // 0 if Kalman filter computed correctly
	  bool verbose; // true to print some information on Serial port
    bool check; // true to check obs data before applying filter, and x estimate after filter

    // UPDATE FILTER WITH OBSERVATION
    void update(BLA::Matrix<Nobs> obs);

    // UPDATE FILTER WITH OBSERVATION and COMMAND
    void update(BLA::Matrix<Nobs> obs, BLA::Matrix<Ncom> com);
    
    // CONSTRUCTOR VERBOSE
    KALMAN<Nstate,Nobs,Ncom>(bool verb); // or verb=true
  
    // CONSTRUCTOR EMPTY (auto set verbose=true)
  	KALMAN<Nstate,Nobs,Ncom>() : KALMAN<Nstate,Nobs,Ncom>(true) {};
  
    // GETTER on X (copy vector to avoid eventual user modifications)
    BLA::Matrix<Nstate> getxcopy();
  
};

/**********      PRIVATE IMPLEMENTATION of UPDATE      **********/

template <int Nstate, int Nobs, int Ncom>
void KALMAN<Nstate,Nobs,Ncom>::_update(BLA::Matrix<Nobs> obs, BLA::Matrix<Nstate> comstate){
  if(check){
    for(int i=0;i<Nobs;i++){
      if(isnan(obs(i)) || isinf(obs(i))){
        if(verbose){Serial.println("KALMAN:ERROR: observation has nan or inf values");}
        status = 1;
        return;
      }
    }
  }
  // UPDATE
  x = F*x+comstate;
  P = F*P*(~F)+Q;
  // ESTIMATION
  y = obs-H*x;
  S = H*P*(~H)+R;
  K = P*(~H)*S.Inverse(&status);
  if(!status){
    x += K*y;
    P = (Id-K*H)*P;
    if(check){
      for(int i=0;i<Nstate;i++){
        if(isnan(x(i)) || isinf(x(i))){
          if(verbose){Serial.println("KALMAN:ERROR: estimated vector has nan or inf values");}
          status = 1;
          return;
        }
      }
    }
  }else{
    if(verbose){Serial.println("KALMAN:ERROR: could not invert S matrix. Try to reset P matrix.");}
    P.Fill(0.0); // try to reset P. Better strategy?
    K.Fill(0.0);
  }
};


/**********      UPDATE with OBS & COM      **********/
template <int Nstate, int Nobs, int Ncom>
void KALMAN<Nstate,Nobs,Ncom>::update(BLA::Matrix<Nobs> obs, BLA::Matrix<Ncom> com){
  if(check){
    for(int i=0;i<Ncom;i++){
      if(isnan(com(i)) || isinf(com(i))){
        if(verbose){Serial.println("KALMAN:ERROR: command has nan or inf values");}
        status = 1;
        return;
      }
    }
  }  
  _update(obs,B*com);
};

/**********      UPDATE with OBS      **********/

template <int Nstate, int Nobs, int Ncom>
void KALMAN<Nstate,Nobs,Ncom>::update(BLA::Matrix<Nobs> obs){
  _update(obs,NULLCOMSTATE);
};

/**********      CONSTRUCTOR      **********/

template <int Nstate, int Nobs, int Ncom>
KALMAN<Nstate,Nobs,Ncom>::KALMAN(bool verb){
  verbose = verb;
  check = true;
  if(verbose){
    Serial.println("KALMAN:INFO: init <"+String(Nstate)+","+String(Nobs)+"> filter");
    if((Nstate<=1)||(Nobs<=1)){
      Serial.println("KALMAN:ERROR: 'Nstate' and 'Nobs' must be > 1");
    }
  }
  P.Fill(0.0);
  x.Fill(0.0);
  Id.Fill(1.0);
  NULLCOMSTATE.Fill(0.0);
};

/**********      GETXCOPY      **********/

template <int Nstate, int Nobs, int Ncom>
BLA::Matrix<Nstate> KALMAN<Nstate,Nobs,Ncom>::getxcopy(){
  BLA::Matrix<Nstate> out;
  for(int i=0;i<Nstate;i++){
    out(i) = x(i);
  }
  return out;
};


#endif
