#pragma once

#include <math/Vector3.h>
#include <math/Matrix.h>

namespace aeroforge::rocket
{
	class RocketMassProperties
	{
	public:
		RocketMassProperties(
			double dryMassKg,
			const math::Vector3& dryCenterOfMassB,
			const math::Matrix& dryInertiaTensorBody,
			const math::Vector3& propellantLocationB
		);

		double getDryMassKg() const;

		double getTotalMassKg(double propellantMassKg) const;

		math::Vector3 getCenterOfMassB(double propellantMassKg) const;

		math::Matrix getInertiaTensorBody(double propellantMassKg) const;

	private:
		double dryMassKg;
		math::Vector3 dryCenterOfMassB;
		math::Matrix dryInertiaTensorBody;
		math::Vector3 propellantLocationB;
	};
}