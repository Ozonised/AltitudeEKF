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
	memset(Xcap, 0, sizeof(Xcap));
	memset(X, 0, sizeof(X));
	memset(Pcap, 0, sizeof(Pcap));
	memset(P, 0, sizeof(P));
	memset(Q, 0, sizeof(Q));
}

/**
 * @brief Sets the predicted (a priori) state vector.
 *
 * @param accelAlt       Predicted altitude in meters.
 * @param accelVelocity  Predicted vertical velocity in m/s.
 *
 * @note This function sets the a priori state (Xcap),
 *       not the corrected/posterior state.
 */
void AltitudeEKF::SetPredictedState(float accelAlt, float accelVelocity)
{
	Xcap[0] = accelAlt;
	Xcap[1] = accelVelocity;
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
 */
void AltitudeEKF::Run(float accelerationZ, float baroAltitude)
{
	// ---- PREDICTION ---- //

	// predicted altitude from acceleration
	Xcap[0] = Xcap[0] + Xcap[1] * dt + 0.5f * accelerationZ * dt * dt;

	// current velocity from acceleration
	Xcap[1] = Xcap[1] + accelerationZ * dt;

	// Predicted Covariance: Pcap = F * P * Ft + Q
	// Calculating Pcap = F * P * Ft:
	Pcap[0][0] = P[0][0] + dt * P[1][0] + (P[0][1] + dt * P[1][1]) * dt;	Pcap[0][1] = P[0][1] + dt * P[1][1];
	Pcap[1][0] = P[1][0] + dt * P[1][1];									Pcap[1][1] = P[1][1];

	// Pcap = P + Q
	Pcap[0][0] += Q[0];
	Pcap[1][1] += Q[1];

	// ---- MEASUREMENT ---- //
	float altError = baroAltitude - Xcap[0];

	// Measurement Prediction Covariance : S = H * Pcap * Ht + R
	float S = Pcap[0][0] + R;
	if (fabs(S) <= 1e-7f) S += copysignf(1e-7f, S);

	// Kalman gain: Kg = Pcap * Ht * S^-1
	float Kg[2];
	Kg[0] = Pcap[0][0] / S;
	Kg[1] = Pcap[1][0] / S;

	// Estimated state: X = Xcap + Kg * altError
	X[0] = Xcap[0] + Kg[0] * altError;
	X[1] = Xcap[1] + Kg[1] * altError;

	// Estimated Covariance: P = (I2 - Kg * H) * Pcap
	P[0][0] = (1.0f - Kg[0]) * Pcap[0][0];		P[0][1] = (1.0f - Kg[0]) * Pcap[0][1];
	P[1][0] = -Kg[1] * Pcap[0][0] + Pcap[1][0];	P[1][1] = -Kg[1] * Pcap[0][1] + Pcap[1][1];
}

