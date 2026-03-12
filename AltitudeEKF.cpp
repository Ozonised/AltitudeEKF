/*
 * AltitudeEKF.cpp
 *
 *  Created on: Feb 11, 2026
 *      Author: farhan
 */

#include "AltitudeEKF.hpp"

AltitudeEKF::AltitudeEKF()
{
	R = 0.0f;
	memset(X, 0, sizeof(X));
	memset(Pcap, 0, sizeof(Pcap));
	memset(P, 0, sizeof(P));
	memset(Q, 0, sizeof(Q));
	P[0][0] = 0.5f;
	P[1][1] = 0.5f;
}

/**
 * @brief Sets the state vector.
 *
 * @param accelAlt       Predicted altitude in meters.
 * @param accelVelocity  Predicted vertical velocity in m/s.
 */
void AltitudeEKF::SetStateVector(float accelAlt, float accelVelocity)
{
	X[0] = accelAlt;
	X[1] = accelVelocity;
}

/**
 * @brief Sets the predicted (a priori) covariance matrix.
 *
 * Assigns the diagonal elements of the predicted covariance matrix.
 *
 * @param P00  Predicted altitude variance.
 * @param P11  Predicted vertical velocity variance.
 *
 * @note Only diagonal terms are assigned. Off-diagonal
 *       covariance terms are ignored in this implementation.
 */
void AltitudeEKF::SetPredictedCovariance(float P00, float P11)
{
	Pcap[0][0] = P00;
	Pcap[1][1] = P11;
}

/**
 * @brief Sets the process noise covariance values.
 *
 * Assigns the diagonal elements of the process noise matrix Q.
 *
 * @param accelAlt       Process noise variance affecting altitude.
 * @param accelVelocity  Process noise variance affecting velocity.
 */
void AltitudeEKF::SetProcessNoise(float accelAlt, float accelVelocity)
{
	Q[0] = accelAlt;
	Q[1] = accelVelocity;
}

/**
 * @brief Sets the measurement noise variance.
 *
 * Assigns the measurement noise covariance R representing
 * barometer altitude uncertainty.
 *
 * @param val  Measurement variance (barometer altitude variance).
 */
void AltitudeEKF::SetMeasurementNoise(float val)
{
	R = val;
}

/**
 * @brief Returns the current estimated altitude.
 *
 * @return Corrected (posterior) altitude estimate in meters.
 *
 * @note This value corresponds to the state after
 *       the Kalman update step.
 */
float AltitudeEKF::GetAltitude(void) const
{
	return X[0];
}

/**
 * @brief Sets the sampling period using the sampling frequency.
 *
 * @param freq  Sampling frequency in Hz.
 */
void AltitudeEKF::SetSamplingTime(float freq)
{
	if (freq != 0.0f)
		dt = (1.0f / freq);
}

/**
 * @brief Executes one complete EKF cycle (prediction + correction).
 *
 * Performs:
 *  - State prediction using vertical acceleration input
 *  - Covariance prediction
 *  - Measurement update using barometric altitude
 *
 * @param accelerationZ  Vertical acceleration input (m/s²).
 *                       Gravity-compensated acceleration is expected.
 * @param baroAltitude   Measured altitude from barometer (meters).
 *
 * @warning If acceleration is not gravity-compensated,
 *          altitude will drift significantly.
 *
 * @note Use Predict(float accelerationZ) and Update(float baroAltitude)
 * 		 when accelerometer and barometer are running at different rate.
 */
void AltitudeEKF::Run(float accelerationZ, float baroAltitude)
{
	// ---- PREDICTION ---- //

	// predicted altitude from acceleration
	 X[0] += X[1] * dt + 0.5f * accelerationZ * dt * dt;

	// current velocity from acceleration
	 X[1] += accelerationZ * dt;

	// Predicted Covariance: Pcap = F * P * Ft + Q
	// Calculating Pcap = F * P * Ft:
	Pcap[0][0] = P[0][0] + dt * P[1][0] + (P[0][1] + dt * P[1][1]) * dt;	Pcap[0][1] = P[0][1] + dt * P[1][1];
	Pcap[1][0] = P[1][0] + dt * P[1][1];									Pcap[1][1] = P[1][1];

	// Pcap = P + Q
	Pcap[0][0] += Q[0];
	Pcap[1][1] += Q[1];

	// ---- MEASUREMENT ---- //
	float altError = baroAltitude - X[0];

	// Measurement Prediction Covariance : S = H * Pcap * Ht + R
	float S = Pcap[0][0] + R;
	if (fabs(S) <= 1e-7f) S += copysignf(1e-7f, S);

	// Kalman gain: Kg = Pcap * Ht * S^-1
	float Kg[2];
	Kg[0] = Pcap[0][0] / S;
	Kg[1] = Pcap[1][0] / S;

	// Estimated state: X = X + Kg * altError
	X[0] += Kg[0] * altError;
	X[1] += Kg[1] * altError;

	// Estimated Covariance: P = (I2 - Kg * H) * Pcap
	P[0][0] = (1.0f - Kg[0]) * Pcap[0][0];		P[0][1] = (1.0f - Kg[0]) * Pcap[0][1];
	P[1][0] = -Kg[1] * Pcap[0][0] + Pcap[1][0];	P[1][1] = -Kg[1] * Pcap[0][1] + Pcap[1][1];
}

/**
 * @brief Executes the prediction step of the EKF.
 *
 * @param accelerationZ  Gravity-compensated vertical acceleration (m/s²).
 *
 * @warning Acceleration must be expressed in the global
 *          vertical frame and properly gravity-compensated.
 *          Otherwise altitude drift will occur.
 */
void AltitudeEKF::Predict(float accelerationZ)
{
	// ---- PREDICTION ---- //

	// predicted altitude from acceleration
	 X[0] += X[1] * dt + 0.5f * accelerationZ * dt * dt;

	// current velocity from acceleration
	 X[1] += accelerationZ * dt;

	// Predicted Covariance: Pcap = F * P * Ft + Q
	// Calculating Pcap = F * P * Ft:
	Pcap[0][0] = P[0][0] + dt * P[1][0] + (P[0][1] + dt * P[1][1]) * dt;	Pcap[0][1] = P[0][1] + dt * P[1][1];
	Pcap[1][0] = P[1][0] + dt * P[1][1];									Pcap[1][1] = P[1][1];

	// Pcap = P + Q
	Pcap[0][0] += Q[0];
	Pcap[1][1] += Q[1];

    memcpy(P, Pcap, sizeof(P));
}

/**
 * @brief Executes the measurement (correction) step of the EKF.
 *
 * Corrects the predicted state using barometric altitude measurement.
 *
 * @param baroAltitude  Measured altitude from barometer (meters).
 *
 */
void AltitudeEKF::Update(float baroAltitude)
{
	// ---- MEASUREMENT ---- //
	float altError = baroAltitude - X[0];

	// Measurement Prediction Covariance : S = H * Pcap * Ht + R
	float S = Pcap[0][0] + R;
	if (fabs(S) <= 1e-7f) S += copysignf(1e-7f, S);

	// Kalman gain: Kg = Pcap * Ht * S^-1
	float Kg[2];
	Kg[0] = Pcap[0][0] / S;
	Kg[1] = Pcap[1][0] / S;

	// Estimated state: X = X + Kg * altError
	X[0] += Kg[0] * altError;
	X[1] += Kg[1] * altError;

	// Estimated Covariance: P = (I2 - Kg * H) * Pcap
	P[0][0] = (1.0f - Kg[0]) * Pcap[0][0];		P[0][1] = (1.0f - Kg[0]) * Pcap[0][1];
	P[1][0] = -Kg[1] * Pcap[0][0] + Pcap[1][0];	P[1][1] = -Kg[1] * Pcap[0][1] + Pcap[1][1];
}
