#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

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
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

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

  // Not initialized until first process measurement
  is_initialized_ = false;

  // Set state dimension
  n_x_ = 5;

  // Initial n_aug_
  n_aug_= 7;

  // Initial sigma point spreading parameter
  lambda_ = 0;

  // Matrix to hold sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Vector for weights
  weights_ = VectorXd(2*n_aug_+1);

  // Start time
  time_us_ = 0;

  // Set NIS
  NIS_radar_ = 0;
  NIS_lidar_ = 0;
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
        if (meas_package.sensor_type_== MeasurementPackage::RADAR){
            double rho = meas_package.raw_measurements_(0);
            double phi = meas_package.raw_measurements_(1);
            double rhodot = meas_package.raw_measurements_(2);

// TODO (wuying#1#): if there is radar information how can i define this 5 dimension vector for state vector
            // state vector[posx, posy,vel,yaw,yawd]
            x_ << rho * cos(phi), rho * sin(phi), 4, rhodot * cos(phi), rhodot * sin(phi);

            P_ << 1,0,0,0,0,
                  0,1,0,0,0,
                  0,0,1,0,0,
                  0,0,0,1,0,
                  0,0,0,0,1;

        }
        else if (meas_package.sensor_type_==MeasurementPackage::LASER){
            x_<<meas_package.raw_measurements_(0),meas_package.raw_measurements_(1),4,0.5,0;
            P_<<1,0,0,0,0,
                0,1,0,0,0,
                0,0,1,0,0,
                0,0,0,1,0,
                0,0,0,0,1;

        }

        is_initialized_=true;
        // Initial timestamp
        time_us_=meas_package.timestamp_;
        return;
  }



  double delta_t = (meas_package.timestamp_- time_us_)/1000000.0;
// TODO (wuying#1#): why we should redefine this time_us again, that means time_us_ is equal to the current time??
  time_us_=meas_package.timestamp_;
  Prediction(delta_t);
// Judge the input is Radar or Lidar
  if (meas_package.sensor_type_==MeasurementPackage::LASER){
        UpdateLidar(meas_package);
 }else if (meas_package.sensor_type_==MeasurementPackage::RADAR){
        UpdateRadar(meas_package);
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

  // in this section you should set sigma point and then use the sigma point to predict the k+1 x

  //// define sigma point matrix for (5,15)
  //MatrixXd Xsig = MatrixXd(n_x_,2*n_aug_+1);

  //// define A for n_x_ dimension
  //MatrixXd A = P_.llt().matrixL();

 // // feed data into Xsig
 // Xsig.col(0)=x_;

  //// calculate sigma point for x
  //lambda_=3-n_x_;

 // for (int i=0;i<n_x_;i++)
 // {
  //    Xsig.col(i+1)=x_+std::sqrt(lambda_+n_x_)*A.col(i);
   //   Xsig.col(i+1+n_x_)=x_-std::sqrt(lambda_+n_x_)*A.col(i);
  //}

  // augment sigma point
  // define sigma points for augment state vector(7)
  VectorXd Xsig_aug_col=VectorXd(n_aug_);

  // define sigma points for augment state matrix(7,15)
  MatrixXd Xsig_aug = MatrixXd(n_aug_,2*n_aug_+1);

  // define sigma points for augment state matrix(7,15)
  // Define covariance matrix for augment sigma point
  MatrixXd Psig_aug=MatrixXd(n_aug_,n_aug_);

  // Construct Xsig_aug
  Xsig_aug_col.head(5) = x_;
  Xsig_aug_col(5)=0;
  Xsig_aug_col(6)=0;

  //create augmented covariance matrix
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0,
        0, std_yawdd_*std_yawdd_;
  Psig_aug.fill(0.0);
  Psig_aug.topLeftCorner(5,5)=P_;
  Psig_aug.bottomRightCorner(2, 2) = Q;

  // Calculate square root of Psig_aug
  MatrixXd Asig = Psig_aug.llt().matrixL();

  // Set scaling factor lambda for augment
  lambda_=3-n_aug_;

  Xsig_aug.col(0)=Xsig_aug_col;

  for (int i=0;i< n_aug_; i++)
  {
      Xsig_aug.col(i+1) = Xsig_aug_col + std::sqrt(lambda_+n_aug_)*Asig.col(i);
      Xsig_aug.col(i+1+n_aug_)=Xsig_aug_col-std::sqrt(lambda_+n_aug_)*Asig.col(i);
  }


  /*********************************************************
  * Predict sigma points

  **********************************************************/

  for (int i = 0; i<2*n_aug_+1;i++) // it is row operation, therefore emulate 15
    {
        VectorXd Xsig_aug_col = Xsig_aug.col(i);
        //double px = Xsig_aug_col(0);
        //double py = Xsig_aug_col(1);
        double vel = Xsig_aug_col(2);
        double yaw = Xsig_aug_col(3);
        double yawd = Xsig_aug_col(4);
        double v_aug = Xsig_aug_col(5);
        double v_yawdd = Xsig_aug_col(6);

        VectorXd vec1=VectorXd(5);
        VectorXd vec2=VectorXd(5);
        VectorXd orig = Xsig_aug_col.head(5);

        // avoid division by zero
        if (yawd > 0.001)
            {
                vec1 << (sin(yaw+yawd*delta_t)-sin(yaw))*vel/yawd,
                       (-cos(yaw+yawd*delta_t)+cos(yaw))*vel/yawd,
                       0,
                       yawd*delta_t,
                       0;

        }
        else {
              vec1 << vel*cos(yaw)*delta_t,
              vel*sin(yaw)*delta_t,
              0,
              yawd*delta_t,
              0;}



        vec2 << .5*v_aug*cos(yaw)*delta_t*delta_t,
               .5*v_aug*sin(yaw)*delta_t*delta_t,
               v_aug*delta_t,
               .5*v_yawdd*delta_t*delta_t,
               v_yawdd*delta_t;


        Xsig_pred_.col(i)<< orig+vec1+vec2;}

   /*********************************************************
  * Calculate mean and covariance

  **********************************************************/
    // construct predicted state vector
  VectorXd x_pred = VectorXd(n_x_);

//construct predicted matrix
  MatrixXd P_pred = MatrixXd(n_x_,n_x_);

// x_diff between x_pred and Xsig_aug_pred

// initial predicted vector and predicted matrix
  x_pred.fill(0.0);
  P_pred.fill(0.0);

// calculate weights
  for (int i =0; i < 2*n_aug_+1; i++){
        if (i ==0){
            weights_(i)=lambda_/(lambda_+n_x_);
        }
        else {
            weights_(i) = .5 /(lambda_ + n_x_);
            }
        // Calculate predicted vector
        x_pred += weights_(i)*Xsig_pred_.col(i);
    }


  //VectorXd x_diff = VectorXd(n_x_);

  for (int i = 0; i< 2*n_aug_+1; i++){
    VectorXd x_diff = Xsig_pred_.col(i)-x_pred;

        // normalize angles
    if (x_diff(3)> M_PI){
            x_diff(3)-=2.*M_PI;
        }
    else if (x_diff(3)<-M_PI){
            x_diff(3)+=2.*M_PI;
        }

    P_pred += weights_(i)*x_diff*x_diff.transpose();
    }

  x_ = x_pred;
  P_ = P_pred;
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
  // Calculate sigma point for measure vector, as for Lidar, there are two parameters to measure[px,py], therefore, sigma point for measure vector is also define [px,py]
/*********************************************************

  * Measurement prediction, mean and covariance

  **********************************************************/
    int n_z= 2;

  //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_+1);

  // Create mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

  // Create matrix for predicted covariance in measure space
    MatrixXd S_pred = MatrixXd(n_z,n_z);

  // initial
    Zsig.fill(0.0);
    z_pred.fill(0.0);
    S_pred.fill(0.0);

  // Calculate predicted measurement mean
    for (int i = 0; i< 2*n_aug_+1; i++){
        VectorXd cal_vec = Xsig_pred_.col(i);

        double px = cal_vec(0);
        double py = cal_vec(1);

        Zsig.col(i)<<px,
                      py;

        z_pred +=weights_(i)*Zsig.col(i);
    }

  // Calculate predicted measurement covariance

    for (int i = 0; i< 2* n_aug_+1;i++){
        VectorXd z_diff = Zsig.col(i)-z_pred;
        S_pred += weights_(i)*z_diff*z_diff.transpose();
    }

 // Define measure noise matrix
    MatrixXd R = MatrixXd(2,2);
    R<<std_laspx_*std_laspx_,0,
        0,std_laspy_*std_laspy_;

    S_pred+=R;

/*********************************************************

* update state

**********************************************************/
    // Calculate cross-correction between points in state and measurement space
    MatrixXd T = MatrixXd(n_x_,n_z);
    T.fill(0.0);

    for (int i=0; i < 2*n_aug_+1;i++){
        VectorXd x_diff=Xsig_pred_.col(i)-x_;//this x_is the predicted mean, cause Line293 we have defined already
            //normalize angles
        if (x_diff(3) > M_PI) {
          x_diff(3) -= 2. * M_PI;
        }
        else if (x_diff(3) < -M_PI) {
          x_diff(3) += 2. * M_PI;
        }
        VectorXd z_diff = Zsig.col(i)-z_pred;
        T+=weights_(i)*x_diff*z_diff.transpose();
    }

    // calculate Kalman Gain
    MatrixXd K = T * S_pred.inverse();

    // define vector for incoming radar data
    VectorXd z = VectorXd(n_z);

    // feed data into z
    double meas_px = meas_package.raw_measurements_(0);
    double meas_py = meas_package.raw_measurements_(1);

    z << meas_px,
         meas_py;
    //state vector update
    VectorXd z_diff = z- z_pred;
    x_+=K*z_diff;

    // Covariance matrix update
    P_ -= K * S_pred * K.transpose();

    // Calculate NIS_Lidar
    NIS_lidar_= z_diff.transpose() *S_pred.inverse() *z_diff;
    //  you should first notice that normalize angle
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
    /*********************************************************

  * Measurement prediction, mean and covariance

  **********************************************************/

    int n_z = 3;

    // Define sigma point in measurement
    MatrixXd Zsig = MatrixXd(n_z,2*n_aug_+1);


    // Define predicted measurement mean
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S = MatrixXd(n_z,n_z);
    MatrixXd R = MatrixXd(n_z,n_z);
    z_pred.fill(0.0);
    S.fill(0.0);
    // there are three parameters in Zsig(...),this data is calculated from Xsig_pred_

    for (int i =0;i < 2*n_aug_+1;i++){
        VectorXd cal_vector= Xsig_pred_.col(i);

        double px = cal_vector(0);
        double py = cal_vector(1);
        double v = cal_vector(2);
        double yaw = cal_vector(3);
        //double yawd = cal_vector(4);

        double rho = sqrt(px*px+py*py);
        double phi = atan2(py,px);
        double rhodot = (px * cos(yaw)*v+py *sin(yaw)*v)/rho;

        Zsig.col(i) << rho,
                       phi,
                       rhodot;
        if (i ==0){
            weights_(i)=lambda_/(lambda_+n_x_);
        }
        else {

            weights_(i) = .5 /(lambda_ + n_x_);
            }

        // Calculate predicted vector
        z_pred += weights_(i)*Zsig.col(i);
    }

    for (int i = 0; i < 2*n_aug_+1;i++){
        VectorXd z_diff = Zsig.col(i) - z_pred;
        if (z_diff(1)> M_PI){
            z_diff(1) -= 2. * M_PI;
        }
        else if (z_diff(1)< -M_PI){
            z_diff(1) += 2. * M_PI;
        }
        if (i ==0){
            weights_(i)=lambda_/(lambda_+n_x_);
        }else {

            weights_(i) = .5 /(lambda_ + n_x_);
            }
        // Calculate predicted vector
        S += weights_(i)*z_diff*z_diff.transpose();
    }

    R << std_radr_ *std_radr_ ,0,0,
            0,std_radphi_*std_radphi_,0,
            0,0,std_radrd_ *std_radrd_ ;

    S += R;

/*********************************************************

* update state

**********************************************************/
    VectorXd z = VectorXd(n_z);

    double meas_rho = meas_package.raw_measurements_(0);
    double meas_phi = meas_package.raw_measurements_(1);
    double meas_rhodot = meas_package.raw_measurements_(2);

    z << meas_rho,
         meas_phi,
         meas_rhodot;

    // Calculate cross correction between points in state and measurement space you nedd to calculate x_diff and z_diff, please notice that you should normalize angle

    MatrixXd T = MatrixXd(n_x_,n_z);
    T.fill(0.0);
    for (int i=0; i< 2*n_aug_+1;i++){

        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        if (x_diff(3)> M_PI){
            x_diff(3) -= 2.* M_PI;
        }
        else if (x_diff(3)< - M_PI){
            x_diff(3) += 2. * M_PI;
        }

        VectorXd z_diff = Zsig.col(i) - z_pred;

        if (z_diff(1)> M_PI){
            z_diff(1) -= 2.* M_PI;
        }
        else if (z_diff(1)< - M_PI){
            z_diff(1) += 2.* M_PI;
        }

        if (i ==0){
            weights_(i)=lambda_/(lambda_+n_x_);
        }
        else {

            weights_(i) = .5 /(lambda_ + n_x_);
            }
        T += weights_(i) * x_diff * z_diff.transpose();
    }

    // Calculate kalman gain
    MatrixXd K = T * S.inverse();

    // Define z_diff between measurement space and state
    VectorXd z_diff = z -z_pred;

    // State update
    x_ += K * z_diff;

    // Covariance matrix update
    P_ -= K * S * K.transpose();

    // calculate NIS_lidar_
    NIS_radar_= z_diff.transpose()* S.inverse() * z_diff;
}


