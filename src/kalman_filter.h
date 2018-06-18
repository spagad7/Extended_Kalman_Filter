#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>

class KalmanFilter
{
    public:

        // state vector
        Eigen::VectorXd x;

        // state transition matrix
        Eigen::MatrixXd F;

        // state covariance matrix
        Eigen::MatrixXd P;

        // process covariance matrix
        Eigen::MatrixXd Q;

        // measurement matrix
        Eigen::MatrixXd H;

        // Jacobian H Matrix
        Eigen::MatrixXd H_j;

        // h(x) vector for radar
        Eigen::VectorXd h;

        // measurement covariance matrix
        Eigen::MatrixXd R_radar;

        Eigen::MatrixXd R_laser;

        // Identity matrix
        Eigen::MatrixXd I;

        // Tools object
        Tools tools;

        /**
        * Constructor
        */
        KalmanFilter();

        /**
        * Destructor
        */
        virtual ~KalmanFilter();

        /**
        * Init Initializes Kalman filter
        * @param x_in Initial state
        * @param P_in Initial state covariance
        * @param F_in Transition matrix
        * @param H_in Measurement matrix
        * @param R_in Measurement covariance matrix
        * @param Q_in Process covariance matrix
        */
        void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &F_in,
                Eigen::MatrixXd &P_in, Eigen::MatrixXd &Q_in,
                Eigen::MatrixXd &H_laser_in, Eigen::MatrixXd &H_j_in,
                Eigen::VectorXd &h_in, Eigen::MatrixXd &R_radar_in,
                Eigen::MatrixXd &R_laser_in);

            /**
            * Prediction Predicts the state and the state covariance
            * using the process model
            * @param delta_T Time between k and k+1 in s
            */
            void Predict();

            /**
            * Updates the state by using standard Kalman Filter equations
            * @param z The measurement at k+1
            */
            void Update(const Eigen::VectorXd &z);

            /**
            * Updates the state by using Extended Kalman Filter equations
            * @param z The measurement at k+1
            */
            void UpdateEKF(const Eigen::VectorXd &z);

    };

    #endif /* KALMAN_FILTER_H_ */
