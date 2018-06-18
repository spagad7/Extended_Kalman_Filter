#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}


KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Init(VectorXd &x_in, MatrixXd &F_in, MatrixXd &P_in,
                        MatrixXd &Q_in, MatrixXd &H_laser_in, MatrixXd &H_j_in,
                        VectorXd &h_in, MatrixXd &R_radar_in, MatrixXd &R_laser_in)
{
    x = x_in;
    F = F_in;
    P = P_in;
    Q = Q_in;
    H = H_laser_in;
    H_j = H_j_in;
    h = h_in;
    R_radar = R_radar_in;
    R_laser = R_laser_in;
    I = MatrixXd::Identity(H.cols(), H.cols());
}


// Function for prediction step in Kalman Filter
void KalmanFilter::Predict()
{
    x = F * x;
    P = F * P * F.transpose() + Q;
}


// Function for update step for LIDAR in Kalman Filter
void KalmanFilter::Update(const VectorXd &z)
{
    MatrixXd H_tr = H.transpose();
    VectorXd y = z - H*x;
    MatrixXd S = H * P * H_tr + R_laser;
    // check if determinant is zero

    if(S.determinant() == 0)
    {
        // skip update step
        cerr << "Determinant == 0, skipping update step!" << endl;
        return;
    }

    MatrixXd K = P * H_tr * S.inverse();
    x = x + K*y;
    P = (I - K*H)*P;
}


// Function for update step for RADAR in Kalman Filter
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
    // calculate h(x) vector to convert state vector values from
    // cartesian to polar coords
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);
    float pxpy = pow(px, 2) + pow(py, 2);
    // check division by zero

    if(pxpy == 0)
    {
        // skip update step
        cerr << "Division by zero, skipping update step!" << endl;
        return;
    }

    float sq_pxpy = pow(pxpy, 0.5);
    h << sq_pxpy, atan2(py, px), (px*vx + py*vy)/sq_pxpy;

    // Calculate error
    VectorXd y = z - h;
    // set yaw value between -pi and pi
    if(y(1) > M_PI)
        y(1) = y(1) - 2*M_PI;
    else if(y(1) < -M_PI)
        y(1) = y(1) + 2*M_PI;

    // Calculate Jacobian matrix and its transpose
    H_j = tools.CalculateJacobian(x);
    MatrixXd H_j_tr = H_j.transpose();
    MatrixXd S = H_j * P * H_j_tr + R_radar;
    MatrixXd K = P * H_j_tr * S.inverse();

    // Update state vector and covariance matrix
    x = x + K*y;
    P = (I - K*H_j)*P;
}
