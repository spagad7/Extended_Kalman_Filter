#include "FusionEKF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


FusionEKF::FusionEKF() {
    is_initialized = false;

    prev_timestamp = 0;

    // Initialize matrices
    x = VectorXd(4);
    F = MatrixXd(4, 4);
    P = MatrixXd(4, 4);
    Q = MatrixXd(4, 4);
    H_laser = MatrixXd(2, 4);
    H_j = MatrixXd(3, 4);
    h = VectorXd(3);
    R_laser = MatrixXd(2, 2);
    R_radar = MatrixXd(3, 3);

    // Set x vector
    x <<    0, 0, 0, 0;

    // Set F matrix
    F <<    1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // Set P matrix
    P <<    1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    // Set Q matrix
    Q <<    0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;

    // Set H matrix
    H_laser <<  1, 0, 0, 0,
                0, 1, 0, 0;

    // Set H_j Matrix
    H_j <<  0, 0, 0, 0,
            0 ,0, 0, 0,
            0, 0, 0, 0;

    // Set h(x) vector
    h << 0, 0, 0;

    // Measurement covariance matrix - radar
    R_radar <<  0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // Measurement covariance matrix - laser
    R_laser <<  0.0225, 0,
                0, 0.0225;

    // Initialize kalman filter
    ekf.Init(x, F, P, Q, H_laser, H_j, h, R_radar, R_laser);

}


FusionEKF::~FusionEKF() {}


// Function to process measurements
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    // Initialize of state_vector x with the first measurement
    if (!is_initialized)
    {
        prev_timestamp = measurement_pack.timestamp;
        if (measurement_pack.sensor_type == MeasurementPackage::RADAR)
        {
            // Convert radar measurement from polar to cartesian coordinates
            float rho = measurement_pack.raw_measurements(0);
            float theta = measurement_pack.raw_measurements(1);
            ekf.x(0) = rho * cos(theta);
            ekf.x(1) = rho * sin(theta);
        }
        else if (measurement_pack.sensor_type == MeasurementPackage::LASER)
        {
            ekf.x(0) = measurement_pack.raw_measurements(0);
            ekf.x(1) = measurement_pack.raw_measurements(1);
        }

        is_initialized = true;
        return;
    }

    // Kalman Filter - Prediction step
    // Calculate delta_t; convert time from microseconds to seconds
    float dt = (measurement_pack.timestamp - prev_timestamp)/1000000.0;
    prev_timestamp = measurement_pack.timestamp;
    // Update state transition matrix F
    ekf.F(0, 2) = dt;
    ekf.F(1, 3) = dt;
    // Update process noise covariance matrix Q
    float dt4 = pow(dt, 4)/4.0;
    float dt3 = pow(dt, 3)/2.0;
    float dt2 = pow(dt, 2);
    ekf.Q <<    dt4*noise_ax,   0,              dt3*noise_ax,   0,
                0,              dt4*noise_ay,   0,              dt3*noise_ay,
                dt3*noise_ax,   0,              dt2*noise_ax,   0,
                0,              dt3*noise_ay,   0,              dt2*noise_ay;
    // Predict new state
    ekf.Predict();

    // Kalman filter - Update step
    if (measurement_pack.sensor_type == MeasurementPackage::RADAR)
        // Update function for RADAR measurement
        ekf.UpdateEKF(measurement_pack.raw_measurements);
    else
        // Update function for LIDAR measurement
        ekf.Update(measurement_pack.raw_measurements);

    // Print the state vector and covariance matrix
    cout << "x = " << ekf.x << endl;
    cout << "P = " << ekf.P << endl;
    cout << endl;
}
