#include <sensors/GpsModel.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::sensors
{
	GpsModel::GpsModel(
		const math::Vector3& positionBiasN,
		const math::Vector3& positionNoiseStdDevN,
		const math::Vector3& velocityBiasN,
		const math::Vector3& velocityNoiseStdDevN,
		std::uint32_t seed
	)
		: positionBiasN(positionBiasN),
		positionNoiseStdDevN(positionNoiseStdDevN),
		velocityBiasN(velocityBiasN),
		velocityNoiseStdDevN(velocityNoiseStdDevN),
		randomEngine(seed),
		standardNormal(0.0, 1.0)
	{
		if (!positionBiasN.isFinite())
		{
			throw std::invalid_argument("positionBiasN must be finite");
		}

		if (!positionNoiseStdDevN.isFinite())
		{
			throw std::invalid_argument("positionNoiseStdDevN must be finite");
		}

		if (positionNoiseStdDevN.x < 0 || positionNoiseStdDevN.y < 0 || positionNoiseStdDevN.z < 0)
		{
			throw std::invalid_argument("positionNoiseStdDevN cannot contain any negative values");
		}

		if (!velocityBiasN.isFinite())
		{
			throw std::invalid_argument("velocityBiasN must be finite");
		}

		if (!velocityNoiseStdDevN.isFinite())
		{
			throw std::invalid_argument("velocityNoiseStdDevN must be finite");
		}

		if (velocityNoiseStdDevN.x < 0 || velocityNoiseStdDevN.y < 0 || velocityNoiseStdDevN.z < 0)
		{
			throw std::invalid_argument("velocityNoiseStdDevN cannot contain any negative values");
		}
	}

	GpsMeasurement GpsModel::sample(
		double timeS,
		const math::Vector3& truePositionN,
		const math::Vector3& trueVelocityN
	)
	{
		if (!std::isfinite(timeS) || timeS < 0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}

		if (!truePositionN.isFinite())
		{
			throw std::invalid_argument("truePositionN must be finite");
		}

		if (!trueVelocityN.isFinite())
		{
			throw std::invalid_argument("trueVelocityN must be finite");
		}

		double positionX = standardNormal(randomEngine);
		double positionY = standardNormal(randomEngine);
		double positionZ = standardNormal(randomEngine);

		double velocityX = standardNormal(randomEngine);
		double velocityY = standardNormal(randomEngine);
		double velocityZ = standardNormal(randomEngine);

		math::Vector3 positionNoise(
			positionNoiseStdDevN.x * positionX,
			positionNoiseStdDevN.y * positionY,
			positionNoiseStdDevN.z * positionZ
		);

		math::Vector3 velocityNoise(
			velocityNoiseStdDevN.x * velocityX,
			velocityNoiseStdDevN.y * velocityY,
			velocityNoiseStdDevN.z * velocityZ
		);

		math::Vector3 measuredPositionN = truePositionN + positionBiasN + positionNoise;
		math::Vector3 measuredVelocityN = trueVelocityN + velocityBiasN + velocityNoise;

		if (!measuredPositionN.isFinite())
		{
			throw std::overflow_error("measuredPositionN must be finite");
		}

		if (!measuredVelocityN.isFinite())
		{
			throw std::overflow_error("measuredVelocityN must be finite");
		}

		return GpsMeasurement{ timeS, measuredPositionN, measuredVelocityN };
	}
}