#include <environment/RelativeAirVelocity.h>

#include <stdexcept>

namespace aeroforge::environment
{
	math::Vector3 computeRelativeAirVelocityN(
		const math::Vector3& vehicleVelocityN,
		const math::Vector3& windVelocityN)
	{
		if (!vehicleVelocityN.isFinite())
		{
			throw std::invalid_argument("Vehicle velocity must be a finite vector.");
		}

		if (!windVelocityN.isFinite())
		{
			throw std::invalid_argument("Wind velocity must be a finite vector.");
		}

		math::Vector3 airVelocityN = vehicleVelocityN - windVelocityN;

		if (!airVelocityN.isFinite())
		{
			throw std::overflow_error("Computed relative air velocity is not finite.");
		}

		return airVelocityN;
	}
}