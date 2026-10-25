#pragma once

#include <math/Vector3.h>
#include <math/Quaternion.h>

namespace aeroforge::telemetry
{
	struct TelemetrySample
	{
		double timeS;

		math::Vector3 positionN;
		math::Vector3 velocityN;
		math::Quaternion attitudeNB;
		math::Vector3 angularVelocityB;

		math::Vector3 estimatedPositionN;
		math::Vector3 estimatedVelocityN;
		math::Quaternion estimatedAttitudeNB;
		math::Vector3 estimatedAngularVelocityB;

		math::Vector3 attitudeErrorB;
		math::Vector3 commandedMomentB;
		math::Vector3 actualMomentB;

		double throttle;
		double gimbalPitchRad;
		double gimbalYawRad;

		bool controllerSaturated;
	};
}