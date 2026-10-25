#pragma once

#include <environment/WindModel.h>

namespace aeroforge::environment
{
	class ConstantWindModel : public WindModel
	{
	public:
		ConstantWindModel();

		explicit ConstantWindModel(const math::Vector3& windVelocityN);

		math::Vector3 getWindVelocityN(double altitudeM, double timeS) const override;

	private:
		math::Vector3 windVelocityN;
	};
}