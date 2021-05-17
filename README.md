# Extended Kalman Filter Project
In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

## Docker environment setup (ubuntu)

#### 1. simulator

- This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

#### 2. docker environment with uWebSocketIO installed

- docker image can be pulled from [here](https://hub.docker.com/r/udacity/controls_kit). The command to run the image is:

  ```bash
  docker run -it -p 4567:4567 -v $(pwd):/work udacity/control_kits /bin/bash
  ```

  - because `$(pwd)` is used here, you'd better `cd` into project folder and run this command.

#### 3. compile&execute and communicate with simulator

- Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

  ```bash
  mkdir build
  cd build
  cmake ..
  make
  ./ExtendedKF
  # run the downloaded simulator.
  ```

  - after `./ExtendedKF`, `Listening to port 4567` will show.
  - after the simulator is run, `Connected!!!` will show.
  - right now necessary ekf functions haven't been filled, therefore `Segmentation fault (core dumped)` will show.

- Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

  - **INPUT**: values provided by the simulator to the c++ program

    `sensor_measurement` => the measurement that the simulator observed (either lidar or radar)

  - **OUTPUT**: values provided by the c++ program to the simulator

    `estimate_x, estimate_y, rmse_x, rmse_y, rmse_vx, rmse_vy`.

### files and implementation

- All kalman filter functions are implemented in `kalman_filter.h` and `kalman_filter.cpp`.
- The kalman filter and extended kalman filter formulas used in this course can be found [here](https://github.com/youngsend/LearningSelfDrivingCars/blob/master/Self-Driving-Cars_Udacity/Computer-Vision-Deep-Learning-and-Sensor-Fusion/l24-extended-kalman-filters.md).

#### 1. initialization using first measurement from lidar or radar

```c++
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
```

- If sensor type is laser, use measurement to initialize `px, py` in state vector `x_`.
  - using laser measurement covariance to initialize state covariance `P_`.
- If sensor type is radar, calculate `px, py` using `rho, phi`.
  - using radar's `rho` measurement covariance to initialize `px, py`'s covariance in `P_`.

#### 2. predict for linear kalman filter

```c++
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
```

- process covariance `Q_` is used to include covariance coming from probable acceleration, which cannot be caught by constant velocity motion model.
- predict step is the same no matter what the sensor type is.

#### 3. update for linear kalman filter (lidar) and extended kalman filter (radar)

```c++
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z - z_pred;
  UpdateWithYHR(y, H_laser_, R_laser_);
}

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
```

- The difference between kf and ekf update is in the calculation of `y` (difference between sensor measurement and measurement prediction calculated with measurement model), `H` (measurement matrix) and `R` (measurement covariance).

### Debug with gdb within docker container

#### 1. install gdb within docker container

```bash
root@704887aea8b2:# apt-get update
root@704887aea8b2:# apt-get install gdb
```

#### 2. debug with gdb (most basic...)

```bash
root@704887aea8b2:/work/build# gdb ./ExtendedKF 
(gdb) run
# error message, see backtrace to find where in the code the bug is from.
(gdb) back
# exit using quit
(gdb) quit
```

### Unscented kalman filter implementation

- Although not for this project, I also implemented [unscented kalman filter](https://github.com/youngsend/Unscented_Kalman_Filter/blob/master/src/ukf.cpp).
- The initialization, predict step, linear kalman filter update (for lidar/laser) and nonlinear kalman filter update (for radar) are similar with this project.