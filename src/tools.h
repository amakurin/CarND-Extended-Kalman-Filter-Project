#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

namespace Tools
{
  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to map state vector x to radar measurment space.
  */
  VectorXd MapStateToRadar(const VectorXd& x_state);

  /**
  * A helper method to map radar measurment z to state vector space.
  */
  VectorXd MapRadarToState(const VectorXd& z);

  /**
  * A helper method to just return identity of right dims.
  */
  MatrixXd I();

  float NormAngle (float angleRad); 
}

#endif /* TOOLS_H_ */
