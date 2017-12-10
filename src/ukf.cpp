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
UKF::UKF()
{
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
// function to normalize angle
void UKF::NormalizeAngle(double *ang)
{
  while (*ang > M_PI) *ang -= 2. * M_PI;
  while (*ang < -M_PI) *ang += 2. * M_PI;
}


void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if (!is_initialized_)
  {
    // Initialize x_, P_, previous_time, ...
    std::cout << "not initialized" << endl;
    // Initilize covariance matrix P
    P_ = MatrixXd::Identity(n_x_,n_x_);

    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      std::cout << "Initialize Laser" << endl;
      // initialize with laser data and fill teh rest with zeros
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      // solve intialization problem with measurements being (close to) zero
      if (fabs(x_(0)) < EPS and fabs(x_(1)) < EPS)
      {
        x_(0) = EPS;
        x_(1) = EPS;
      }
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
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
    for (int i = 1; i < weights_.size(); i++)
    {
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

  // update Radar and Lidar measurements
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    UpdateRadar(meas_package);
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //std::cout << "prediction step" << endl;

  // delta t squared
  double delta_t2 = delta_t*delta_t;

  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Augmented state covariance matrix P_aug
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // augmented sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  // Fill the matrices
  x_aug.fill(0.0); // fill with zeros
  x_aug.head(n_x_) = x_; // add x_ vector in the first n_x_ positions
  P_aug.fill(0); // fill with zeros
  P_aug.topLeftCorner(n_x_,n_x_) = P_; // add P_ to P_aug

  // add squared process noise
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // L is square root of P matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create sigma points
  Xsig_aug.col(0) = x_aug;
  double sqrt_lambda_n_aug_ = sqrt(lambda_+n_aug_);
  VectorXd sqrt_lambda_n_aug_L;
  for(int i = 0; i < n_aug_; i++)
  {
    sqrt_lambda_n_aug_L          = sqrt_lambda_n_aug_ * L.col(i);
    Xsig_aug.col(i + 1)          = x_aug + sqrt_lambda_n_aug_L;
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt_lambda_n_aug_L;
  }
  
  // predict sigma points
  for (int i = 0; i< n_sig_; i++)
  {
    // save values in variables for clarity
    double p_x      = Xsig_aug(0,i);
    double p_y      = Xsig_aug(1,i);
    double v        = Xsig_aug(2,i);
    double yaw      = Xsig_aug(3,i);
    double yawd     = Xsig_aug(4,i);
    double nu_a     = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // calculate sin, cos, and arg
    double sin_yaw = sin(yaw);
    double cos_yaw = cos(yaw);
    double arg     = yaw + yawd * delta_t;
    
    // create predicted state variables
    double px_p, py_p;

    // calculate predicted state variables when not zero
    if (fabs(yawd) > EPS)
    { 
        px_p = p_x + v/yawd * (sin(arg) - sin_yaw);
        py_p = p_y + v/yawd * (cos_yaw - cos(arg) );
    }
    // ...and when zero
    else
    {
        double v_delta_t = v*delta_t;
        px_p = p_x + v_delta_t*cos_yaw;
        py_p = p_y + v_delta_t*sin_yaw;
    }

    // velocity and yaw rate variables 
    double v_p = v;
    double yaw_p = arg;
    double yawd_p = yawd;

    // add noise
    px_p   += 0.5 * nu_a * delta_t2 * cos_yaw;
    py_p   += 0.5 * nu_a * delta_t2 * sin_yaw;
    v_p    += nu_a * delta_t;
    yaw_p  += 0.5 * nu_yawdd * delta_t2;
    yawd_p += nu_yawdd * delta_t;

    // write predicted sigma points into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  
  // include weights
  x_ = Xsig_pred_ * weights_;

  // fill predicted state covariance matrix with zeros
  P_.fill(0.0);

  // go through all sigma points
  for (int i = 0; i < n_sig_; i++)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //  normalize the angle
    NormalizeAngle(&(x_diff(3)));
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  
  //std::cout << "update step lidar" << endl;

  // measurement dimensions (px, py)
  int n_z = 2;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);

  // update UKF with measurement data
  UpdateUKF(meas_package, Zsig, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //std::cout << "update step radar" << endl;


  // measurement dimension (r, phi, and r_dot)
  int n_z = 3;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++)
  {
    // save values in variables for clarity
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v   = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double v1  = cos(yaw)*v;
    double v2  = sin(yaw)*v;

    // Measurement model
    // r
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
    // phi
    Zsig(1,i) = atan2(p_y,p_x);
    // r_dot
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / Zsig(0,i);
  }

  // update UKF with measurement data
  UpdateUKF(meas_package, Zsig, n_z);
}

// update function
void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z)
{

}
