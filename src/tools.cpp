#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


// Function to calculate Root Mean Square Error
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                                const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    VectorXd temp(4);
    int n = estimations.size();
    if(n == 0)
        return rmse;

    for(int i=0; i<n; i++)
    {
        temp = estimations[i] - ground_truth[i];
        temp = temp.array() * temp.array();
        rmse += temp;
    }
    rmse = rmse/n;
    rmse = rmse.array().sqrt();

    return rmse;
}

//TODO: Consider converting MatrixXd to MatrixXf for more performance
// Function to calculate Jacobian matrix from state vector
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    MatrixXd J = MatrixXd::Zero(3, 4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    float pxpy = pow(px, 2) + pow(py, 2);

    // check division by zero
    if(pxpy == 0)
    {
        cerr << "Division by zero in Jacobian matrix!" << endl;
        return J;
    }

    float sq_pxpy = pow(pxpy, 0.5);
    float cu_pxpy = pow(pxpy, 1.5);
    float num_1 = py*(vx*py - vy*px);
    float num_2 = px*(vy*px - vx*py);
    // Fill Jacobian Matrix
    J   <<  px/sq_pxpy,     py/sq_pxpy,     0,          0,
            -py/pxpy,       px/pxpy,        0,          0,
            num_1/cu_pxpy,  num_2/cu_pxpy,  px/sq_pxpy, py/sq_pxpy;

    return J;
}
