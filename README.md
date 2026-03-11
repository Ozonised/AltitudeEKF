# Altitude EKF
An extended kalman filter to combine accelerometer readings and barometer altitude to estimate altitude.

# AltitudeEKF Usage Guide

## Overview

`AltitudeEKF` is a lightweight Extended Kalman Filter designed to estimate **altitude and vertical velocity** by fusing:

* **Vertical acceleration** from an IMU
* **Altitude measurements** from a barometer

The filter maintains a **2-state model**:

$`X =
\begin{bmatrix}z\\
v_{z}\end{bmatrix}`$

where

* `z` → altitude (meters)
* $`v_{z}`$ → vertical velocity (m/s)

The prediction step integrates acceleration to estimate altitude and velocity, while the measurement update corrects the altitude estimate using barometer data.

---

# Filter Workflow

The EKF operates in two steps:

### 1. Prediction

Uses acceleration to propagate the state forward in time.

```cpp
ekf.Predict(accelZ);
```

### 2. Measurement Update

Corrects the predicted state using barometric altitude.

```cpp
ekf.Update(baroAltitude);
```

For convenience, both operations can be combined:

```cpp
ekf.Run(accelZ, baroAltitude);
```

---

# Initial Setup

Before using the filter, configure its parameters.

```cpp
AltitudeEKF ekf;

// Set sampling frequency
ekf.SetSamplingTime(100.0f);  // 100 Hz

// Initial state estimate
ekf.SetStateVector(initialAltitude, 0.0f);

// Initial covariance (uncertainty)
ekf.SetPredictedCovariance(10.0f, 1.0f);

// Process noise
ekf.SetProcessNoise(Q_altitude, Q_velocity);

// Measurement noise (barometer variance)
ekf.SetMeasurementNoise(R_baro);
```

Recommended approach:

* `Q_altitude` and `Q_velocity` should reflect uncertainty caused by accelerometer noise.
* `R_baro` should be the **variance of barometer altitude measurements**.

---

# Case 1 — Same Sensor Data Rate

If the **accelerometer and barometer run at the same frequency**, the filter can be updated in a single step.

Example: both sensors running at **50 Hz**

```cpp
#include "AltitudeEKF.hpp"

AltitudeEKF ekf;

int main(void)
{
  // Set sampling frequency
  ekf.SetSamplingTime(50.0f);  // 50 Hz
  
  // Initial state estimate
  ekf.SetStateVector(initialAltitude, 0.0f);
  
  // Initial covariance (uncertainty)
  ekf.SetPredictedCovariance(10.0f, 1.0f);
  
  // Process noise
  ekf.SetProcessNoise(0.0001f, 0.0001f);
  
  // Measurement noise (barometer variance)
  ekf.SetMeasurementNoise(0.0121f);

  while (1)
  {
      // Function to get vertical acceleration excluding acceleration due to gravity
      float accelZ = GetVerticalAcceleration();
      float baroAlt = GetBarometerAltitude();
  
      ekf.Run(accelZ, baroAlt);
  
      float altitude = ekf.GetAltitude();
  }
}
```

### Workflow

```
loop:
    read accelerometer
    read barometer
    Run EKF
    get altitude estimate
```

This is the **simplest configuration**.

---

# Case 2 — Different Sensor Data Rates

In many systems:

* IMU → **high rate** (100–1000 Hz)
* Barometer → **low rate** (10–50 Hz)

In this case the EKF should run:

* **Prediction at IMU rate**
* **Update at barometer rate**

Example:

IMU = **200 Hz**
Barometer = **25 Hz**

```cpp
#include "AltitudeEKF.hpp"

AltitudeEKF ekf;

int main(void)
{
  // Set sampling frequency
  ekf.SetSamplingTime(200.0f);  // 200 Hz
  
  // Initial state estimate
  ekf.SetStateVector(initialAltitude, 0.0f);
  
  // Initial covariance (uncertainty), set the values as per your setup
  ekf.SetPredictedCovariance(0.1f, 1.0f);
  
  // Process noise
  ekf.SetProcessNoise(0.0001f, 0.0001f);
  
  // Measurement noise (barometer variance)
  ekf.SetMeasurementNoise(0.0121f);

  while (1)
  {
      // Function to get vertical acceleration excluding acceleration due to gravity
      float accelZ = GetVerticalAcceleration();
  
      // Always run prediction at IMU rate
      ekf.Predict(accelZ);
  
      if (BarometerMeasurementAvailable())
      {
          float baroAlt = GetBarometerAltitude();
          ekf.Update(baroAlt);
      }
  
      float altitude = ekf.GetAltitude();
  }
```

### Workflow

```
IMU interrupt (fast):
    Predict()

Barometer interrupt (slow):
    Update()
```

Advantages:

* High-rate motion tracking
* Low-rate absolute altitude correction
* Improved dynamic response

This architecture is commonly used in **flight controllers and robotics systems**.

---

# Important Notes

### Gravity Compensation

The acceleration input must be **gravity compensated**:

$`[a_z = a_{measured} - g]`$

and expressed in the **world vertical frame**.

Failure to remove gravity will cause altitude drift.

---

### Sensor Frame Alignment

The accelerometer measurement must represent **vertical acceleration in the global frame**.

Typically this requires:

* orientation estimation (e.g., quaternion or AHRS)
* rotating body acceleration into the world frame.

---

### Noise Tuning

Filter performance depends strongly on tuning.

Typical strategy:

* Increase **Q** → faster response but noisier output
* Increase **R** → smoother output but slower correction

---

# Minimal Example

```cpp
AltitudeEKF ekf;

ekf.SetSamplingTime(100.0f);
ekf.SetProcessNoise(0.0001f, 0.0001f);
ekf.SetMeasurementNoise(0.0121f);

while (1)
{
    float accelZ = GetVerticalAcceleration();
    float baroAlt = GetBarometerAltitude();

    ekf.Run(accelZ, baroAlt);

    printf("Altitude: %.2f\n", ekf.GetAltitude());
}
```

---

# Summary

`AltitudeEKF` provides a simple and efficient way to fuse:

* **high-frequency accelerometer data**
* **low-frequency barometric altitude**

to obtain a stable altitude estimate.

Use:

* `Run()` when both sensors operate at the same rate
* `Predict()` + `Update()` when sensors run at different rates.
