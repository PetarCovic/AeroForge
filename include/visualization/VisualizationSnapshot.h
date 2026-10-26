#pragma once

#include <math/Quaternion.h>
#include <math/Vector3.h>

namespace aeroforge::visualization
{
	struct VisualizationSnapshot
	{
		double simulationTimeS;

		math::Vector3 truthPositionN;
		math::Vector3 truthVelocityN;
		math::Quaternion truthAttitudeNB;
		math::Vector3 truthAngularVelocityB;

		math::Vector3 estimatedPositionN;
		math::Vector3 estimatedVelocityN;
		math::Quaternion estimatedAttitudeNB;
		math::Vector3 estimatedAngularVelocityB;

		math::Vector3 attitudeErrorB;

		math::Vector3 commandedMomentB;
		math::Vector3 actualMomentB;
	};
}