#include <iostream>
#include "Eigen/Dense"
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;
  // State dimension
  n_x_ = 5;
  // Augmented state dimension
  n_aug_ = 7;
  // initial state vector
  x_ = VectorXd(n_x_);
  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  // augmented state vector
  x_aug_ = VectorXd(n_aug_);
  // augmented state covariance
  P_aug_ = MatrixXd(n_aug_, n_aug_);
  // sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);
  // predicted sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  // Process noise matrix
  Q_ = MatrixXd(2, 2);
  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
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
  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
  // time when the state is true, in us
  time_us_ = 0;
  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  x_ << 0, 0, 0, 0, 0;

  P_.fill(0.0);
  P_(0,0) = 1;  // px
  P_(1,1) = 1;  // py
  P_(2,2) = 1;  // v
  P_(3,3) = 0.1;  // yaw
  P_(4,4) = 0.1;  // yaw dot

  // predicted sigma points matrix
  Xsig_pred_.fill(0.0);
  // Weights of sigma points
  weights_.fill(0.0);
  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
  // Process noise matrix
  Q_ << std_a_*std_a_, 0,
        0, std_yawdd_*std_yawdd_;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  * @param {MeasurementPackage} meas_package The latest measurement data of
  * either radar or laser.
  */
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "UKF: " << endl;

    double v = 5;  // derived from data file analysis
    double yaw = 0;
    double yaw_d = 0;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "State initialized with RADAR measurement" << endl;
      double x = meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]);
      double y = meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]);
      x_ << x, y, v, yaw, yaw_d;

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      cout << "State initialized with LIDAR measurement" << endl;
      double x = meas_package.raw_measurements_[0];
      double y = meas_package.raw_measurements_[1];
      x_ << x, y, v, yaw, yaw_d;
    }

    // timestamp initialization
    time_us_ = meas_package.timestamp_;

    cout << "x_init = " << x_.transpose() << endl;
    cout << "P_init = " << P_ << endl;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(dt);
  cout << endl << "### Prediction ###" << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if ((meas_package.sensor_type_ == MeasurementPackage::LASER) & use_laser_) {
    cout << endl << "### Update Lidar ###" << endl;
    int update_lidar_method = 1;    // 0 -> UKF, other value -> KF
    UpdateLidar(meas_package.raw_measurements_, &nis_, update_lidar_method);

  } else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR)  & use_radar_) {
    cout << endl << "### Update Radar ###" << endl;
    UpdateRadar(meas_package.raw_measurements_, &nis_);
  }

  // print the output
  cout << "x_ = " << x_.transpose() << endl;
  cout << "P_ = " << P_ << endl;
  cout << "NIS_: " << nis_ << endl;

}

void UKF::Prediction(double delta_t) {
/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
  AugmentedSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
}

void UKF::AugmentedSigmaPoints() {

  //create augmented mean state
  x_aug_.head(n_x_) = x_;
  int n_d = n_aug_-n_x_;
  x_aug_.tail(n_d) = VectorXd::Zero(n_d);

  //create augmented covariance matrix
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_.bottomRightCorner(n_d,n_d) = Q_;

  //create square root matrix
  MatrixXd A = P_aug_.llt().matrixL();
  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  //set remaining sigma points
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i+1)     = x_aug_ + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * A.col(i);
  }
  //print result
  //std::cout << "Xsig_aug_ = " << Xsig_aug_ << std::endl;

}

void UKF::SigmaPointPrediction(double delta_t) {
  /**
   * Generate Augmented Sigma Points with process noise
   */
  for (int i=0;i<2*n_aug_+1;++i) {
    VectorXd x = Xsig_aug_.col(i);
    VectorXd xp =  VectorXd(n_x_);
    if (fabs(x(4)) < 0.001) {  //avoid division by zero
      //cout << "division par 0" << endl;
      xp(0) = x(0) + x(2)*cos(x(3))*delta_t + 0.5*delta_t*delta_t*x(5)*cos(x(3));
      xp(1) = x(1) + x(2)*sin(x(3))*delta_t + 0.5*delta_t*delta_t*x(5)*sin(x(3));
    }
    else {
      xp(0) = x(0) + x(2)/x(4)*(sin(x(3)+x(4)*delta_t)-sin(x(3))) + 0.5*delta_t*delta_t*x(5)*cos(x(3));
      xp(1) = x(1) + x(2)/x(4)*(-cos(x(3)+x(4)*delta_t)+cos(x(3))) + 0.5*delta_t*delta_t*x(5)*sin(x(3));
    }
      xp(2) = x(2) + delta_t*x(5);
      xp(3) = x(3) + x(4)*delta_t + 0.5*delta_t*delta_t*x(6);
      xp(4) = x(4) + delta_t*x(6);
    //write predicted sigma points into right column
    Xsig_pred_.col(i) = xp;
  }

  //print result
  //std::cout << "Xsig_pred_ = " << Xsig_pred_ << std::endl;
}

void UKF::PredictMeanAndCovariance() {
  /**
   * Predict Mean And Covariance
   */

  //set weights
  int n_sigma = 2*n_aug_+1;
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  double weight = 0.5/(n_aug_+lambda_);
  for (int i=1; i<n_sigma; ++i) {  //2n+1 weights
    weights_(i) = weight;
  }

  //create mean vector and covariance matrix for prediction
  VectorXd x = VectorXd::Zero(n_x_);
  MatrixXd P = MatrixXd::Zero(n_x_, n_x_);

  // Compute predicted mean
  for (int i=0; i<n_sigma; ++i) {
    x = x + weights_(i)*Xsig_pred_.col(i);
  }
  // Compute predicted covariance
  for (int i=0; i<n_sigma; ++i) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  //write result
  x_ = x;
  P_ = P;

  //print result

  std::cout << "Predicted state = " ;
  std::cout << x.transpose() << std::endl;
  //std::cout << "Predicted covariance matrix = ";
  //std::cout << P << std::endl;
}

void UKF::UpdateLidar(VectorXd z, double* nis, int update_lidar_method) {
  /**
  Use Lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  Calculate the radar NIS.
  */
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;

  //measurement covariance matrix R
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  if (update_lidar_method == 0) {  // UKF
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);

    //transform sigma points into measurement space
    for (int i=0; i<2*n_aug_+1;++i) {
      Zsig(0,i) = Xsig_pred_(0,i);
      Zsig(1,i) = Xsig_pred_(1,i);
    }
    //calculate mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i<2*n_aug_+1; ++i) {  //iterate over sigma points
      z_pred += weights_(i)*Zsig.col(i);
    }
    //compute predicted sigma measurement covariance matrix
    MatrixXd I = MatrixXd(n_z,n_z);
    I.fill(0.0);
    //compute (Xsig,Zsig) covariance matrix
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    for (int i = 0; i<2*n_aug_+1; ++i) {  //iterate over sigma point
      // measurement difference
      VectorXd z_diff = Zsig.col(i) - z_pred;
      // state difference
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      // phi angle normalization
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

      I = I + weights_(i) * z_diff * z_diff.transpose() ;
      Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    //calculate innovation covariance matrix S
    S = I + R;

    //calculate Kalman gain K;
    MatrixXd K = MatrixXd(n_x_, n_z);
    K = Tc*S.inverse();

    //update state mean and covariance matrix
    x_ = x_ + K*(z-z_pred);
    P_ = P_ - K*S*K.transpose();

  }
  else { // KF
	// measurement matrix definition
	MatrixXd H = MatrixXd(2, 5);
	H << 1, 0, 0, 0, 0,
		 0, 1, 0, 0, 0;

    z_pred = H * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;

  }
  // Compute NIS
  *nis = (z-z_pred).transpose()*S.inverse()*(z-z_pred);

  //print result
  /*
  std::cout << "z_pred:" << z_pred.transpose() << std::endl;
  std::cout << "S: " << S << std::endl;
  std::cout << "K: " << K << std::endl;
  */
}

void UKF::UpdateRadar(VectorXd z, double* nis) {
  /**
  Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  Calculate the radar NIS.
  */
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //measurement covariance matrix R
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;

  //transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1;++i) {
    Zsig(0,i) = sqrt(Xsig_pred_(0,i)*Xsig_pred_(0,i) + Xsig_pred_(1,i)*Xsig_pred_(1,i));
    Zsig(1,i) = atan2(Xsig_pred_(1,i),Xsig_pred_(0,i));
    Zsig(2,i) = (Xsig_pred_(0,i)*Xsig_pred_(2,i)*cos(Xsig_pred_(3,i))+Xsig_pred_(1,i)*Xsig_pred_(2,i)*sin(Xsig_pred_(3,i)))/Zsig(0,i);
  }
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i<2*n_aug_+1; ++i) {  //iterate over sigma points
    z_pred += weights_(i)*Zsig.col(i);
  }
  //compute predicted sigma measurement covariance matrix
  MatrixXd I = MatrixXd(n_z,n_z);
  I.fill(0.0);
  //compute (Xsig,Zsig) covariance matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i<2*n_aug_+1; ++i) {  //iterate over sigma point
    // measurement difference
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // phi angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    I = I + weights_(i) * z_diff * z_diff.transpose() ;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  //calculate innovation covariance matrix S
  S = I + R;

  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x_, n_z);
  K = Tc*S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //print result
  /*
  std::cout << "z_pred: " << z_pred << std::endl;
  std::cout << "S: " << S << std::endl;
  std::cout << "K: " << K << std::endl;
  */
  // Compute NIS
  *nis = z_diff.transpose()*S.inverse()*z_diff;
}
