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
