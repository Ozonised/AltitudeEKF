/*
 * AltitudeEKF.hpp
 *
 *  Created on: Feb 11, 2026
 *      Author: farhan
 */

#ifndef ALTITUDEEKF_HPP_
#define ALTITUDEEKF_HPP_

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>

class AltitudeEKF
{
	private:
		float Xcap[2];		// state vector/predicted state
		float X[2];			// estimated state
		float Pcap[2][2];	// predicted covariance
		float Q[2];			// Process noise
		float P[2][2];		// Estimated covariance
		float R;			// Measurement noise (barometers noise)
		float dt;
	public:
		AltitudeEKF();
		void SetSamplingTime(float freq);
		void SetPredictedState(float accelAlt, float accelVelocity);
		void SetPredictedCovariance(float P00, float P11);
		void SetProcessNoise(float accelAlt, float accelVelocity);
		void SetMeasurementNoise(float val);
		void Run(float accelerationZ, float baroAltitude);
		float GetAltitude(void) const;
};


#endif /* ALTITUDEEKF_ALTITUDEEKF_HPP_ */
