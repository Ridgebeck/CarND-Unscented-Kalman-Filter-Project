#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPS 0.0001

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30; //1.5

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30; //0.57
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:
  Complete the initialization. See ukf.h for other member properties.


  Hint: one or more values initialized above might be wildly off...
  */
  // initialization state
  is_initialized_ = false;

  // time stamp in micro seconds
  time_us_ = 0;

  // state dimension
  n_x_ = x_.size();

  // augmented state dimension
  n_aug_ = n_x_ + 2;

  // number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  // spreading parameter for sigma points
  lambda_ = 3 - n_aug_;

  // weights of sigma points
  weights_ = VectorXd(n_sig_); 

  // matrix with predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // noise covariance matrix for radar measurements
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;
  
  // noise covariance matrix for lidar measurements
  R_lidar_ = MatrixXd(2,2);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;

  std::cout << "initializiation done" << endl;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_){
    // Initialize x_, P_, previous_time, ...
    std::cout << "not initialized" << endl;
    // Initilize covariance matrix P
    P_ = MatrixXd::Identity(n_x_,n_x_);

    if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      std::cout << "Initialize Laser" << endl;
      // initialize with laser data and fill teh rest with zeros
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      // solve intialization problem with measurements being (close to) zero
      if (fabs(x_(0)) < EPS and fabs(x_(1)) < EPS){
        x_(0) = EPS;
        x_(1) = EPS;
      }
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
      std::cout << "Initialize Radar" << endl;

      // save measurement values in variables for easier calculation of teh conversion
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[2];

      // convert coordinates from polar to cartesian
      float px = rho * cos(phi); 
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      float v  = sqrt(vx * vx + vy * vy);
      x_ << px, py, v, 0, 0;
    }

    // Initialize sigma weights
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < weights_.size(); i++) {
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }
    
    // read and save initial timestamp
    time_us_ = meas_package.timestamp_;

    // set flag after initialization is done
    is_initialized_ = true;

    return;
  }

  // calculate the time between measurements
  double dt = (meas_package.timestamp_ - time_us_);
  // convert us to s
  dt /= 1000000.0;
  // read and save new timestamp
  time_us_ = meas_package.timestamp_;
  // predict sigma points, the state, and the state covariance matrix for dt
  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    //cout << "Radar " << meas_package.raw_measurements_[0] << " " << meas_package.raw_measurements_[1] << endl;
      UpdateRadar(meas_package);
    }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    //cout << "Lidar " << meas_package.raw_measurements_[0] << " " << meas_package.raw_measurements_[1] << endl;
      UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  std::cout << "prediction step" << endl;


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  std::cout << "update step lidar" << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  std::cout << "update step radar" << endl;

}
