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
