#include <environment/ConstantWindModel.h>

#include <stdexcept>

namespace aeroforge::environment
{
	ConstantWindModel::ConstantWindModel()
		: windVelocityN(0.0, 0.0, 0.0)
	{
	}

	ConstantWindModel::ConstantWindModel(const math::Vector3& windVelocityN)
		: windVelocityN(windVelocityN)
	{
		if (!windVelocityN.isFinite())
		{
			throw std::invalid_argument("Wind velocity must be a finite vector.");
		}
	}

	math::Vector3 ConstantWindModel::getWindVelocityN(double altitudeM, double timeS) const
	{
		(void)altitudeM; // Unused parameter
		(void)timeS;     // Unused parameter

		return windVelocityN;
	}
} // namespace aeroforge::environment