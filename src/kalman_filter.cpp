#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
   
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;

  /* Remaining calculations are common to Kalman and EKF update */
    UpdateMatrices(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  /* Convert the predicted state from cartesian to polar co-ordinates to compute error */
    float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    float theta = atan2(x_(1) , x_(0));                    // atan2 returns value between -pi,pi
    
    if(rho < 0.00001)
    {
      rho = 0.00001;
    }
    
    float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    VectorXd h = VectorXd(3); // h(x_)
    h << rho, theta, rho_dot;

    /***************** Debug info ****************************/
    std::cout << " theta: " << z[1] << std::endl;
    std::cout << "theta pred " << theta << std::endl; 
    /***************** End ****************************/
    
    VectorXd y = z - h;
    
    /***************** Debug info ****************************/
    std::cout << "theta diff " << y[1] << std::endl; 
    /***************** End ****************************/

    /* Normalize computed error in theta so that it is also within -pi,pi range */
    y[1] = atan2(sin(y[1]),cos(y[1]));

    /***************** Debug info ****************************/
    std::cout << "theta diff normalized " << y[1] << std::endl; 
    /***************** End ****************************/

    /* Remaining calculations are common to Kalman and EKF update */
    UpdateMatrices(y);

}

/*  Update Kalman Filter steps common to EKF and KF update. Equations from the lectures */
void KalmanFilter::UpdateMatrices(const VectorXd &y){
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
  // New state
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
