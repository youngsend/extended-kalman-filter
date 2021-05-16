#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "measurement_package.h"

class KalmanFilter {
public:
  KalmanFilter();
  ~KalmanFilter() = default;

  /**
 * Run the whole flow of the Kalman Filter from here.
 */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(float dt);

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

  // common part for kf and ekf update.
  // only y, H, R are different.
  void UpdateWithYHR(const Eigen::VectorXd &y,
                     const Eigen::MatrixXd &H,
                     const Eigen::MatrixXd &R);

  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  // calculate z_pred using nonlinear radar measurement model.
  Eigen::VectorXd CalculateRadarPrediction(const Eigen::VectorXd &x_state);
  // normalize angle in y (z-h(x)) so that it's within -pi ~ pi.
  float NormalizeAngle(float angle_rad);

  Eigen::VectorXd GetStateVector() {return x_;};

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_{false};

  // previous timestamp
  long long previous_timestamp_{0};

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix, will be calculated based on noise_ax, noise_ay, elapsed_time.
  Eigen::MatrixXd Q_;
  float noise_ax_{5}, noise_ay_{5};

  // measurement matrix
  Eigen::MatrixXd H_;

  // laser measurement covariance matrix
  Eigen::MatrixXd R_laser_;
  // radar measurement covariance matrix
  Eigen::MatrixXd R_radar_;
  // laser measurement matrix
  Eigen::MatrixXd H_laser_;
  // H_radar_/Hj (Jacobian matrix is calculated within kalman_filter.cpp because it changes.)
};

#endif // KALMAN_FILTER_H_
