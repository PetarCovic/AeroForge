#pragma once

#include <math/Vector3.h>

#include <cstdint>
#include <random>

namespace aeroforge::sensors
{
	struct GpsMeasurement
	{
		double timeS;
		math::Vector3 positionN;
		math::Vector3 velocityN;
	};

	class GpsModel
	{
	public:
		GpsModel(
			const math::Vector3& positionBiasN,
			const math::Vector3& positionNoiseStdDevN,
			const math::Vector3& velocityBiasN,
			const math::Vector3& velocityNoiseStdDevN,
			std::uint32_t seed
		);

		GpsMeasurement sample(
			double timeS,
			const math::Vector3& truePositionN,
			const math::Vector3& trueVelocityN
		);

	private:
		math::Vector3 positionBiasN;
		math::Vector3 positionNoiseStdDevN;

		math::Vector3 velocityBiasN;
		math::Vector3 velocityNoiseStdDevN;

		std::mt19937 randomEngine;
		std::normal_distribution<double> standardNormal;
	};
}