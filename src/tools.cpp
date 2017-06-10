#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

namespace Tools
{
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                                const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    int estimationsSize = estimations.size();

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimationsSize != ground_truth.size()
          || estimationsSize == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }
    //accumulate squared residuals
    for(int i=0; i < estimationsSize; ++i){
        VectorXd diff = estimations[i]-ground_truth[i];
        VectorXd residual = diff.array()*diff.array();
        rmse += residual;
    }
    rmse = rmse/estimationsSize;
    rmse = rmse.array().sqrt();
    return rmse;
  }

  MatrixXd CalculateJacobian(const VectorXd& x_state) {
    /**
    TODO:
      * Calculate a Jacobian here.
    */
    MatrixXd J(3,4);

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //precalculate denominators 
    float d2 = px*px+py*py;
    float d = sqrt(d2);
    float d3 = (d2*d);

    //check division by zero
    if (d < 0.000001){
        cout<<"WARNING! Division by zero! d ="<<d<<endl;
        J << 0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 0;
    }
    else{
        float H01 = px/d;
        float H02 = py/d;
        J << H01, H02, 0, 0,
            -py/d2, px/d2, 0, 0,
            py*(vx*py-vy*px)/d3, px*(vy*px-vx*py)/d3, H01, H02; 
    }

    return J;
  }

  VectorXd MapStateToRadar(const VectorXd& x_state){
    VectorXd mapped(3);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float rho = sqrt(px*px + py*py);
    float phi = atan2(py,px);
    mapped << rho, phi, (px*vx + py*vy)/rho;

    return mapped;
  }

  VectorXd MapRadarToState(const VectorXd& z){
    VectorXd mapped(4);

    float rho = z(0);
    float phi = z(1);
    float drho = z(2);

    float px = (rho*cos(phi));
    float py = (rho*sin(phi));
    float vx = (rho*cos(phi));
    float vy = (rho*sin(phi));
    mapped << px, py, vx, vy;

    return mapped;
  }

  MatrixXd I(){
    return MatrixXd::Identity(4, 4);
  }

  float NormAngle (float angleRad) {
    if (fabs(angleRad) > M_PI) {
        float pi2 = (M_PI * 2.);
        float correction = (floor((angleRad + M_PI) / pi2) * pi2);
        angleRad -= correction;
    }
    return angleRad;
  }

}

