#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
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

    /**
    TODO:
      * Finish initializing the FusionEKF.
      * Set the process and measurement noises
    */
    MatrixXd P(4, 4);
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    ekf_.P_ = P;

    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

    VectorXd z = measurement_pack.raw_measurements_;

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
            float ro = z[0];    // radial distance from the origin
            float theta = z[1]; // angle between rho and x
            ekf_.x_ << ro * cos(theta), ro * sin(theta), 0, 0;

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            ekf_.x_ << z[0], z[1], 0, 0;
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;
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
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    MatrixXd F(4, 4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

    MatrixXd G(4, 2);
    G << dt*dt/2.0, 0,
         0, dt*dt/2.0,
         dt, 0,
         0, dt;

    MatrixXd Qv(2, 2);
    Qv << 9.0, 0,
          0, 9.0;

    MatrixXd Q(4, 4);
    Q = G * Qv * G.transpose();

    ekf_.F_ = F;
    ekf_.Q_ = Q;
    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
}
