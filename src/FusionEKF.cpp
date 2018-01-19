#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // measurement matrix - laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    /**
     TODO:
     * Finish initializing the FusionEKF.
     * Set the process and measurement noises
     */
    
    // State vector
    ekf_.x_ = VectorXd(4);
    
    // State Co-variance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
    
    // Initial Transition Matrix
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ <<  1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;
    
    // Process Covariance Matrix
    ekf_.Q_ = MatrixXd(4, 4);
    
    // Acceleration Noise Components
    noise_ax = 9;
    noise_ay = 9;
    
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float range = measurement_pack.raw_measurements_[0];
      float bearing = measurement_pack.raw_measurements_[1];

      // Feeding the first received data
      ekf_.x_ << range*cos(bearing), range*sin(bearing), 0.0, 0.0;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      //Initialize state.
      

      // Feeding the first received data
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
    }
      
    // Updating the timestamp of last recorded data
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   // Calculating difference between timestamps dt in seconds
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;

   // Checking whether time has elapsed or not
   // Skipping prediction if no time has elapsed a
   if (dt > 0)
   {
     // Updating F matrix as per dt
     ekf_.F_(0,2) = dt;
     ekf_.F_(1,3) = dt;

     // Updating Q matrix
     double t1 = pow(dt, 4)/4;
     double t2 = pow(dt, 3)/2;
     double t3 = pow(dt, 2);

     ekf_.Q_ <<  t1*noise_ax, 0, t2*noise_ax, 0,
                0, t1*noise_ay, 0, t2*noise_ay,
                t2*noise_ax, 0, t3*noise_ax, 0,
                0, t2*noise_ay, 0, t3*noise_ay;

     // Predicting the state and co-variance matrix
     ekf_.Predict();
   }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  // If RADAR sensor measurement
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    float px = ekf_.x_[0];
    float py = ekf_.x_[1];
      
    // Checking value of px and py to avoid dividing by 0 error for Hj calculation
    if (!(px == 0 && py == 0))
    {
        // Radar updates
        
        // Calculating Jacobian matrix Hj
        Hj_ = tools.CalculateJacobian(ekf_.x_);
          
        // Updating H & R matrix for RADAR measurement
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
        
        // Updating with RADAR measurements
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    }
  }
  // If LASER sensor measurement
  else
  {
    // Laser updates
     
    // Updating H & R for LASER measurements
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    // Updating with LASER measurements
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}


