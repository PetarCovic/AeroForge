#pragma once

#include <math/Vector3.h>

#include <cstdint>
#include <random>

namespace aeroforge::sensors
{
	class AccelerometerModel
	{
	public:
		AccelerometerModel(
			const math::Vector3& biasB,
			const math::Vector3& noiseStdDevB,
			const math::Vector3& saturationAbsB,
			std::uint32_t seed
		);

		math::Vector3 measure(
			const math::Vector3& trueSpecificForceB
		);

	private: 
		math::Vector3 biasB;
		math::Vector3 noiseStdDevB;
		math::Vector3 saturationAbsB;

		std::mt19937 randomEngine;
		std::normal_distribution<double> standardNormal;
	};
}