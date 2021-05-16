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

