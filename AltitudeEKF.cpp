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

void AltitudeEKF::SetPredictedState(float accelAlt, float accelVelocity)
{
	Xcap[0] = accelAlt;
	Xcap[1] = accelVelocity;
}

void AltitudeEKF::SetPredictedCovariance(float P00, float P11)
{
	Pcap[0][0] = P00;
	Pcap[1][1] = P11;
}

void AltitudeEKF::SetProcessNoise(float accelAlt, float accelVelocity)
{
	Q[0] = accelAlt;
	Q[1] = accelVelocity;
}

void AltitudeEKF::SetMeasurementNoise(float val)
{
	R = val;
}

float AltitudeEKF::GetAltitude(void) const
{
	return X[0];
}

void AltitudeEKF::SetSamplingTime(float freq)
{
	dt = (1.0f / freq);
}

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

