#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"
#include <cmath>

class FusionEKF {
    public:
        KalmanFilter ekf;

        /**
        * Constructor.
        */
        FusionEKF();

        /**
        * Destructor.
        */
        virtual ~FusionEKF();

        /**
        * Run the whole flow of the Kalman Filter from here.
        */
        void ProcessMeasurement(const MeasurementPackage &measurement_pack);

    private:
        // check whether the tracking toolbox was initialized or not (first measurement)
        bool is_initialized;
        // previous timestamp
        long long prev_timestamp;
        // noise values for Process Covariance Matrix (Q)
        int noise_ax = 9;
        int noise_ay = 9;

        // tool object used to compute Jacobian and RMSE
        Tools tools;
        Eigen::VectorXd x;
        Eigen::MatrixXd F;
        Eigen::MatrixXd P;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd H_laser;
        Eigen::MatrixXd H_j;
        Eigen::VectorXd h;
        Eigen::MatrixXd R_laser;
        Eigen::MatrixXd R_radar;
};

#endif /* FusionEKF_H_ */
