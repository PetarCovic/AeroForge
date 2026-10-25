#include <sensors/GyroscopeModel.h>

#include <stdexcept>
#include <algorithm>

namespace aeroforge::sensors
{
	GyroscopeModel::GyroscopeModel(
		const math::Vector3& biasB,
		const math::Vector3& noiseStdDevB,
		const math::Vector3& saturationAbsB,
		std::uint32_t seed
	)
		: biasB(biasB), noiseStdDevB(noiseStdDevB), saturationAbsB(saturationAbsB), randomEngine(seed), standardNormal(0.0, 1.0)
	{
		if (!biasB.isFinite())
		{
			throw std::invalid_argument("biasB must be finite");
		}

		if (!noiseStdDevB.isFinite())
		{
			throw std::invalid_argument("noiseStdDevB must be finite and non-negative");
		}

		if (noiseStdDevB.x < 0 || noiseStdDevB.y < 0 || noiseStdDevB.z < 0)
		{
			throw std::invalid_argument("noiseStdDevB cannot contain any negative values");
		}

		if (!saturationAbsB.isFinite())
		{
			throw std::invalid_argument("saturationAbsB must be finite");
		}

		if (saturationAbsB.x <= 0 || saturationAbsB.y <= 0 || saturationAbsB.z <= 0)
		{
			throw std::invalid_argument("saturationAbsB must contain only positive numbers");
		}
	}

	math::Vector3 GyroscopeModel::measure(
		const math::Vector3& trueAngularVelocityB
	)
	{
		if (!trueAngularVelocityB.isFinite())
		{
			throw std::invalid_argument("trueAngularVelocityB must be finite");
		}

		double nx = standardNormal(randomEngine);
		double ny = standardNormal(randomEngine);
		double nz = standardNormal(randomEngine);

		math::Vector3 measurement;

		measurement.x = trueAngularVelocityB.x + biasB.x + noiseStdDevB.x * nx;
		measurement.y = trueAngularVelocityB.y + biasB.y + noiseStdDevB.y * ny;
		measurement.z = trueAngularVelocityB.z + biasB.z + noiseStdDevB.z * nz;

		measurement.x = std::clamp(measurement.x, -saturationAbsB.x, saturationAbsB.x);
		measurement.y = std::clamp(measurement.y, -saturationAbsB.y, saturationAbsB.y);
		measurement.z = std::clamp(measurement.z, -saturationAbsB.z, saturationAbsB.z);

		if (!measurement.isFinite())
		{
			throw std::overflow_error("measurement must be finite");
		}

		return measurement;
	}
}