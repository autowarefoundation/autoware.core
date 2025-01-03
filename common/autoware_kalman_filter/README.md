# kalman_filter

## Overview

This package contains the kalman filter with time delay and the calculation of the kalman filter.

## Design

The Kalman filter is a recursive algorithm used to estimate the state of a dynamic system. The Time Delay Kalman filter is based on the standard Kalman filter and takes into account possible time delays in the measured values.

### Standard Kalman Filter

#### System Model

Assume that the system can be represented by the following linear discrete model:

$$
x_{k} = A x_{k-1} + B u_{k} \\
y_{k} = C x_{k-1}
$$

where,

- $x_k$ is the state vector at time $k$.
- $u_k$ is the control input vector at time $k$.
- $y_k$ is the measurement vector at time $k$.
- $A$ is the state transition matrix.
- $B$ is the control input matrix.
- $C$ is the measurement matrix.

#### Prediction Step

The prediction step consists of updating the state and covariance matrices:

$$
x_{k|k-1} = A x_{k-1|k-1} + B u_{k} \\
P_{k|k-1} = A P_{k-1|k-1} A^{T} + Q
$$

where,

- $x_{k|k-1}$ is the priori state estimate.
- $P_{k|k-1}$ is the priori covariance matrix.

#### Update Step

When the measurement value \( y_k \) is received, the update steps are as follows:

$$
K_k = P_{k|k-1} C^{T} (C P_{k|k-1} C^{T} + R)^{-1} \\
x_{k|k} = x_{k|k-1} + K_k (y_{k} - C x_{k|k-1}) \\
P_{k|k} = (I - K_k C) P_{k|k-1}
$$

where,

- $K_k$ is the Kalman gain.
- $x_{k|k}$ is the posterior state estimate.
- $P_{k|k}$ is the posterior covariance matrix.

### Extension to Time Delay Kalman Filter

For the Time Delay Kalman filter, it is assumed that there may be a maximum delay of step ($d$) in the measured value. To handle this delay, we extend the state vector to:

$$
(x_{k})_e = \begin{bmatrix}
x_k \\
x_{k-1} \\
\vdots \\
x_{k-d+1}
\end{bmatrix}
$$

The corresponding state transition matrix ($A_e$) and process noise covariance matrix ($Q_e$) are also expanded:

$$
A_e = \begin{bmatrix}
A & 0 & 0 & \cdots & 0 \\
I & 0 & 0 & \cdots & 0 \\
0 & I & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 0
\end{bmatrix}, \quad
Q_e = \begin{bmatrix}
Q & 0 & 0 & \cdots & 0 \\
0 & 0 & 0 & \cdots & 0 \\
0 & 0 & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 0
\end{bmatrix}
$$

#### Prediction Step

The prediction step consists of updating the extended state and covariance matrices.

Update extension status:

$$
(x_{k|k-1})_e = \begin{bmatrix}
A & 0 & 0 & \cdots & 0 \\
I & 0 & 0 & \cdots & 0 \\
0 & I & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 0
\end{bmatrix}
\begin{bmatrix}
x_{k-1|k-1} \\
x_{k-2|k-1} \\
\vdots \\
x_{k-d|k-1}
\end{bmatrix}
$$

Update extended covariance matrix:

$$
(P_{k|k-1})_e = \begin{bmatrix}
A & 0 & 0 & \cdots & 0 \\
I & 0 & 0 & \cdots & 0 \\
0 & I & 0 & \cdots & 0 \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 0
\end{bmatrix}
\begin{bmatrix}
P_{k-1|k-1}^{(1)} & P_{k-1|k-1}^{(1,2)} & \cdots & P_{k-1|k-1}^{(1,d)} \\
P_{k-1|k-1}^{(2,1)} & P_{k-1|k-1}^{(2)} & \cdots & P_{k-1|k-1}^{(2,d)} \\
\vdots & \vdots & \ddots & \vdots \\
P_{k-1|k-1}^{(d,1)} & P_{k-1|k-1}^{(d,2)} & \cdots & P_{k-1|k-1}^{(d)}
\end{bmatrix}
\begin{bmatrix}
 A^T & I & 0 & \cdots & 0 \\
 0 & 0 & I & \cdots & 0 \\
 0 & 0 & 0 & \cdots & 0 \\
 \vdots & \vdots & \vdots & \ddots & \vdots \\
 0 & 0 & 0 & \cdots & 0
 \end{bmatrix} +
 \begin{bmatrix}
 Q & 0 & 0 & \cdots & 0 \\
 0 & 0 & 0 & \cdots & 0 \\
 0 & 0 & 0 & \cdots & 0 \\
 \vdots & \vdots & \vdots & \ddots & \vdots \\
 0 & 0 & 0 & \cdots & 0
 \end{bmatrix}
$$

$\Longrightarrow$

$$
(P_{k|k-1})_e = \begin{bmatrix} A P_{k-1|k-1}^{(1)} A^T + Q & A P_{k-1|k-1}^{(1,2)} & \cdots & A P_{k-1|k-1}^{(1,d)} \\ P_{k-1|k-1}^{(2,1)} A^T & P_{k-1|k-1}^{(2)} & \cdots & P_{k-1|k-1}^{(2,d)} \\ \vdots & \vdots & \ddots & \vdots \\ P_{k-1|k-1}^{(d,1)} A^T & P_{k-1|k-1}^{(d,2)} & \cdots & P_{k-1|k-1}^{(d)} \end{bmatrix}
$$

where,

- $(x_{k|k-1})_e$ is the priori extended state estimate.
- $(P_{k|k-1})_e$ is the priori extended covariance matrix.

#### Update Step

When receiving the measurement value ( $y_{k}$ ) with a delay of ( $ds$ ), the update steps are as follows:

Update kalman gain:

$$
K_k = \begin{bmatrix}
P_{k|k-1}^{(1)} C^T \\
P_{k|k-1}^{(2)} C^T \\
\vdots \\
P_{k|k-1}^{(ds)} C^T \\
\vdots \\
P_{k|k-1}^{(d)} C^T
\end{bmatrix}
(C P_{k|k-1}^{(ds)} C^T + R)^{-1}
$$

Update extension status:

$$
(x_{k|k})_e = \begin{bmatrix}
x_{k|k-1} \\
x_{k-1|k-1} \\
\vdots \\
x_{k-d+1|k-1}
\end{bmatrix} +
\begin{bmatrix}
K_k^{(1)} \\
K_k^{(2)} \\
\vdots \\
K_k^{(ds)} \\
\vdots \\
K_k^{(d)}
\end{bmatrix} (y_k - C x_{k-ds|k-1})
$$

Update extended covariance matrix:

$$
 (P_{k|k})_e = \left(I -
 \begin{bmatrix}
 K_k^{(1)} C \\
 K_k^{(2)} C \\
 \vdots \\
 K_k^{(ds)} C \\
 \vdots \\
 K_k^{(d)} C
 \end{bmatrix}\right)
 \begin{bmatrix}
 P_{k|k-1}^{(1)} & P_{k|k-1}^{(1,2)} & \cdots & P_{k|k-1}^{(1,d)} \\
 P_{k|k-1}^{(2,1)} & P_{k|k-1}^{(2)} & \cdots & P_{k|k-1}^{(2,d)} \\
 \vdots & \vdots & \ddots & \vdots \\
 P_{k|k-1}^{(d,1)} & P_{k|k-1}^{(d,2)} & \cdots & P_{k|k-1}^{(d)}
 \end{bmatrix}
$$

where,

- $K_k$ is the Kalman gain.
- $(x_{k|k})_e$ is the posterior extended state estimate.
- $(P_{k|k})_e$ is the posterior extended covariance matrix.
- $C$ is the measurement matrix, which only applies to the delayed state part.

## Example Usage

This section describes Example Usage of KalmanFilter.

- Initialization

```cpp
#include "autoware/kalman_filter/kalman_filter.hpp"

// Define system parameters
int dim_x = 2; // state vector dimensions
int dim_y = 1; // measure vector dimensions

// Initial state
Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(dim_x, 1);
x0 << 0.0, 0.0;

// Initial covariance matrix
Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(dim_x, dim_x);
P0 *= 100.0;

// Define state transition matrix
Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x, dim_x);
A(0, 1) = 1.0;

// Define measurement matrix
Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x);
C(0, 0) = 1.0;

// Define process noise covariance matrix
Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(dim_x, dim_x);
Q *= 0.01;

// Define measurement noise covariance matrix
Eigen::MatrixXd R = Eigen::MatrixXd::Identity(dim_y, dim_y);
R *= 1.0;

// Initialize Kalman filter
autoware::kalman_filter::KalmanFilter kf;
kf.init(x0, P0);
```

- Predict step

```cpp
const Eigen::MatrixXd x_next = A * x0;
kf.predict(x_next, A, Q);
```

- Update step

```cpp
// Measured value
Eigen::MatrixXd y = Eigen::MatrixXd::Zero(dim_y, 1);
kf.update(y, C, R);
```

- Get the current estimated state and covariance matrix

```cpp
Eigen::MatrixXd x_curr = kf.getX();
Eigen::MatrixXd P_curr = kf.getP();
```

## Assumptions / Known limits

- Delay Step Check: Ensure that the `delay_step` provided during the update does not exceed the maximum delay steps set during initialization.
