#include "kalman_filter.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    x = x_in;
    P = P_in;
    F = F_in;
    H = H_in;
    R = R_in;
    Q = Q_in;
    I = MatrixXd::Identity(H.cols(), H.cols());
}

void KalmanFilter::Predict()
{
    x = F * x;
    P = F * P * F.transpose();
}

void KalmanFilter::Update(const VectorXd &z)
{
    MatrixXd H_tr = H.transpose();
    VectorXd y = z - H*x;
    MatrixXd S = H * P * H_tr + R;
    MatrixXd K = P * H_tr * S.inverse();
    x = x + K*y;
    P = (I - K*H)*P;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
    // calculate h(x) vector to convert state vector values in cartesian coords
    // to polar coords
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);
    float pxpy = pow(px, 2) + pow(py, 2);
    float sq_pxpy = pow(pxpy, 0.5);
    VectorXd h(3);
    h << sq_pxpy, atan2(py, px), (px*vx + py*vy)/sq_pxpy;

    MatrixXd H_j_tr = H_j.transpose();
    VectorXd y = z - h*x;
    // set yaw value between -pi and pi
    if(y(1) > PI)
        y(1) = y(1) - 2*PI;
    else if(y(1) < -PI)
        y(1) = y(1) + 2*PI;
    MatrixXd S = H_j * P * H_j_tr + R;
    MatrixXd K = P * H_j_tr * S.inverse();
    x = x  + K*y;
    P = (I - K*H)*P;
}
