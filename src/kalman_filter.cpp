#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
  // state vector, we don't know yet the values of the x state
  x_ = VectorXd(4);

  // state covariance matrix P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
          0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
          0, 0.0009, 0,
          0, 0, 0.09;

  // measurement matrix - laser
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
          0, 1, 0, 0;

  // the initial transition matrix. because elapsed_time is in transition matrix, it will change.
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

  // process covariance matrix. Q_ will be recalculated in Predict function, because of delta_t.
  Q_ = MatrixXd(4, 4);
}

void KalmanFilter::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // initialize state with measurement if uninitialized.
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      // use laser measurement covariance to initialize px, py covariance.
      P_(0, 0) = R_laser_(0, 0);
      P_(1, 1) = R_laser_(1, 1);
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      auto rho = measurement_pack.raw_measurements_[0];
      auto phi = measurement_pack.raw_measurements_[1];
      x_ << rho * cos(phi), rho * sin(phi), 0, 0;
      // use radar rho measurement covariance to initialize px, py covariance.
      P_(0, 0) = R_radar_(0, 0);
      P_(1, 1) = R_radar_(0, 0);
    }
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;

    // initialize kalman filter
    return;
  }

  /**
   * Prediction
   */
  float delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  Predict(delta_t);
  previous_timestamp_ = measurement_pack.timestamp_;

  /**
   * Update
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    Update(measurement_pack.raw_measurements_);
  }
}

/**
 * predict step for linear kalman filter
 */
void KalmanFilter::Predict(float dt) {
  // 1. modify the F_ matrix so that the time is integrated.
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // 2. set the process covariance matrix Q
  Q_ << pow(dt,4)/4 * noise_ax_, 0, pow(dt,3)/2 * noise_ax_, 0,
        0, pow(dt,4)/4 * noise_ay_, 0, pow(dt,3)/2 * noise_ay_,
        pow(dt,3)/2 * noise_ax_, 0, pow(dt,2) * noise_ax_, 0,
        0, pow(dt,3)/2 * noise_ay_, 0, pow(dt,2) * noise_ay_;

  // 3. use kalman filter predict
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

/**
 * update step for linear kalman filter
 * @param z
 */
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateWithYHR(y, H_, R_laser_);
}

/**
 * update step for extended kalman filter
 * @param z
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // use H_jacobian rather than H_.
  MatrixXd Hj = CalculateJacobian(x_);
  VectorXd z_pred = CalculateRadarPrediction(x_);
  VectorXd y = z - z_pred;
  // normalize angle in y.
  y(1) = NormalizeAngle(y(1));
  UpdateWithYHR(y, Hj, R_radar_);
}

void KalmanFilter::UpdateWithYHR(const Eigen::VectorXd &y,
                                 const Eigen::MatrixXd &H,
                                 const Eigen::MatrixXd &R) {
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;
}

/**
 * calculate radar measurement function's jacobian, against state vector x.
 * @param x_state
 * @return
 */
MatrixXd KalmanFilter::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  // check division by zero
  if (fabsf(c1) < 0.0001) {
    std::cout << "CalculateJacobian() [Error] division by zero\n";
    return Hj;
  }

  // compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}

Eigen::VectorXd KalmanFilter::CalculateRadarPrediction(const Eigen::VectorXd &x_state) {
  VectorXd z_pred(3);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c2 = sqrtf(px*px+py*py);
  // check division by zero
  if (fabsf(c2) < 0.0001) {
    std::cout << "CalculateRadarPrediction() [Error] division by zero\n";
    return z_pred;
  }

  // compute radar measurements using radar measurement model.
  z_pred << c2, std::atan2(py, px), (px*vx+py*vy)/c2;

  return z_pred;
}

float KalmanFilter::NormalizeAngle(float angle_rad) {
  // angle normalization
  while (angle_rad > M_PI)
    angle_rad -= 2*M_PI;
  while (angle_rad < -M_PI)
    angle_rad += 2*M_PI;
  return angle_rad;
}
