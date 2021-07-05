#include "ukf.h"
#include "Eigen/Dense"
#include "iostream"
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = false;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.0;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  is_initialized_ =  false;
  n_x_ = 5 ; // px, py, v, yaw, yaw_dot
  n_aug_ = 7; //n_x_ + noise va, noise v_yaw_dot_dot
  lambda_ = 3-n_aug_;
  Xsig_pred_ = MatrixXd(n_x_,2 * n_aug_ + 1);
  //creation weights;
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  double weight = 0.5/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; ++i) {  
    weights_(i) = weight;
  }

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  // initialize  the state based on the first   measurements
  if(!is_initialized_){
    if(meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER){
      x_ << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1),0,0,0;
      P_ << std_laspx_*std_laspx_, 0, 0,  0,  0,
            0, std_laspy_*std_laspy_, 0,  0,  0,
            0,  0,  1,  0,  0,
            0,  0,  0,  1,  0,
            0,  0,  0,  0,  1;
    }
    else{
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];
      double px = rho * cos(phi);
      double py  = rho * sin(phi);
      double v = rho_dot;  // assumption
      x_ << px, py, v, 0, 0;

      P_ << std_radr_*std_radr_, 0, 0,  0,  0,
                  0, std_radr_*std_radr_, 0,  0,  0,
                  0,  0,  std_radrd_*std_radrd_,  0,  0,
                  0,  0,  0,  std_radphi_*std_radphi_,  0,
                  0,  0,  0,  0,  std_radphi_*std_radphi_;
    }
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
    return;
  }
  //delta t
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  //Prediction
  //  Motion Model, sensor has no impact
  Prediction(delta_t);

  //Update
  // Lidar//Radar
  if(meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER){
    UpdateLidar(meas_package);
  }else if(meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR){
    UpdateRadar(meas_package);
  }

}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */


  // Step1: predict augumented  points

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_); //7
  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_,n_aug_); //7x7
  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1); //7x15


  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //Step 2: Sigma points prediction

  // predict sigma points
  for (int i = 0; i< 2*n_aug_+1; ++i) {
    // extract values for better readability
    double p_x =  Xsig_aug(0,i);
    double p_y =  Xsig_aug(1,i);
    double v   =  Xsig_aug(2,i);
    double yaw =  Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    } else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  // Step 3 : sigmapoints mean and covariance


  // predicted state mean
  x_.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }

  //


}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  int n_z = 2;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_(0),meas_package.raw_measurements_(1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0);


  // transform sigma points into measu  rement space
   z_pred.fill(0.0);

  for(int i=0;i<2*n_aug_+1;i++){
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      Zsig(0,i) = px; //p_x
      Zsig(1,i)  = py; //p_y
      z_pred +=  weights_(i) *  Zsig.col(i);
  }
  // calculate mean predicted measurement
  // calculate innovation covariance matrix S
  
  for(int i=0 ; i<2*n_aug_+1;i++){
      VectorXd s_diff = Zsig.col(i) - z_pred;
         // angle normalization
      S += weights_(i) * s_diff * s_diff.transpose(); 
  }
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0);
  R(0,0) = std_laspx_ * std_laspx_;
  R(1,1) = std_laspy_ * std_laspy_; 
  S = S + R;


  MatrixXd Tc = MatrixXd(n_x_, n_z);

// calculate cross correlation matrix
    Tc.fill(0.0);
    for(int i=0; i<2*n_aug_+1;i++){
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)< -M_PI) x_diff(3)+=2.*M_PI;

        VectorXd z_diff = Zsig.col(i) - z_pred;
        Tc += weights_(i) * x_diff  * z_diff.transpose();
    }
    

  // calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
    VectorXd z_diff = z- z_pred;
    x_ =x_+ K*z_diff;
    P_ = P_ - K*S*K.transpose();
    NIS_lidar = z_diff.transpose() * S.inverse() * z_diff;

    //std::cout<< x_ << std::endl;
    //std::cout << "Lidar " <<x_.array()<< std::endl;
    std::cout<< "Lidar NIS_lidar"<<NIS_lidar<<std::endl;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  int n_z = 3;
  VectorXd z = VectorXd(n_z);
  double rho = meas_package.raw_measurements_(0);
  double phi = meas_package.raw_measurements_(1);
  double rhod = meas_package.raw_measurements_(2);
  z << rho, phi, rhod;
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1); //3x15

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0);


  // transform sigma points into measurement space
   z_pred.fill(0.0);

  for(int i=0;i<2*n_aug_+1;i++){
      double px = Xsig_pred_(0,i);
      double py = Xsig_pred_(1,i);
      double v = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      Zsig(0,i) = sqrt(px*px  + py*py); //roh
      Zsig(1,i)  = atan2(py,px); //phi
      Zsig(2,i) = (px*cos(yaw)*v + py*sin(yaw)*v) /  Zsig(0,i); // roh_dot
      z_pred +=  weights_(i) *  Zsig.col(i);
  }
  // calculate mean predicted measurement
  // calculate innovation covariance matrix S
  
  for(int i=0 ; i<2*n_aug_+1;i++){
      VectorXd s_diff = Zsig.col(i) - z_pred;
         // angle normalization
      while (s_diff(1)> M_PI) s_diff(1)-=2.*M_PI;
      while (s_diff(1)<-M_PI) s_diff(1)+=2.*M_PI;
      S += weights_(i) * s_diff * s_diff.transpose(); 
  }
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_; 
  R(2,2) = std_radrd_ * std_radrd_;

  S = S + R;


  MatrixXd Tc = MatrixXd(n_x_, n_z);

// calculate cross correlation matrix
    Tc.fill(0.0);
    for(int i=0; i<2*n_aug_+1;i++){
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)< -M_PI) x_diff(3)+=2.*M_PI;

        VectorXd z_diff = Zsig.col(i) - z_pred;
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)< -M_PI) z_diff(1)+=2.*M_PI;
        Tc += weights_(i) * x_diff  * z_diff.transpose();
    }
    

  // calculate Kalman gain K;
    MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
    VectorXd z_diff = z - z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)< -M_PI) z_diff(1)+=2.*M_PI;
    x_ =x_+ K*z_diff;
    P_ = P_ - K*S*K.transpose();
    //std::cout<< "Radar"<<x_.array()<<std::endl;
    //std::cout<< "Radar"<<P_.array()<<std::endl;
    NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
    std::cout<< "Radar NIS_radar"<<NIS_radar<<std::endl;


}