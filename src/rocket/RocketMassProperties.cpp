#include <rocket/RocketMassProperties.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::rocket
{
	RocketMassProperties::RocketMassProperties(
		double dryMassKg,
		const math::Vector3& dryCenterOfMassB,
		const math::Matrix& dryInertiaTensorBody,
		const math::Vector3& propellantLocationB
	)
		: dryMassKg(dryMassKg), dryCenterOfMassB(dryCenterOfMassB), dryInertiaTensorBody(dryInertiaTensorBody), propellantLocationB(propellantLocationB)
	{
		if (!std::isfinite(dryMassKg) || dryMassKg<=0)
		{
			throw std::invalid_argument("dryMassKg must be finite and positive");
		}

		if (!dryCenterOfMassB.isFinite())
		{
			throw std::invalid_argument("dryCenterOfMassB must be finite");
		}

		if (dryInertiaTensorBody.rows() != 3 || dryInertiaTensorBody.columns() != 3)
		{
			throw std::invalid_argument("dryInertiaTensorBody must be 3x3");
		}

		if (!dryInertiaTensorBody.isFinite())
		{
			throw std::invalid_argument("dryInertiaTensorBody must be finite");
		}

		if (!dryInertiaTensorBody.isSymmetric())
		{
			throw std::invalid_argument("dryInertiaTensorBody must be symmetric");
		}

		if (!dryInertiaTensorBody.isPositiveDefinite())
		{
			throw std::invalid_argument("dryInertiaTensorBody must be positive definite");
		}

		if (!propellantLocationB.isFinite())
		{
			throw std::invalid_argument("propellantLocationB must be finite");
		}
	}

	double RocketMassProperties::getDryMassKg() const
	{
		return dryMassKg;
	}

	double RocketMassProperties::getTotalMassKg(double propellantMassKg) const
	{
		if (!std::isfinite(propellantMassKg) || propellantMassKg<0)
		{
			throw std::invalid_argument("propellantMassKg must be finite and non-negative");
		}

		return dryMassKg + propellantMassKg;
	}

	math::Vector3 RocketMassProperties::getCenterOfMassB(double propellantMassKg) const
	{
		if (!std::isfinite(propellantMassKg) || propellantMassKg < 0)
		{
			throw std::invalid_argument("propellantMassKg must be finite and non-negative");
		}

		math::Vector3 com = (dryMassKg * dryCenterOfMassB + propellantMassKg * propellantLocationB) / (dryMassKg + propellantMassKg);

		if (!com.isFinite())
		{
			throw std::overflow_error("CenterOfMass must be finite");
		}

		return com;
	}

	math::Matrix RocketMassProperties::getInertiaTensorBody(double propellantMassKg) const
	{
		if (!std::isfinite(propellantMassKg) || propellantMassKg < 0)
		{
			throw std::invalid_argument("propellantMassKg must be finite and non-negative");
		}

		math::Vector3 com = getCenterOfMassB(propellantMassKg);
		math::Vector3 dryOffsetB = dryCenterOfMassB - com;

		math::Vector3 propellantOffsetB = propellantLocationB - com;

		double x=dryOffsetB.x;
		double y=dryOffsetB.y;
		double z=dryOffsetB.z;

		math::Matrix dryShift(
			3,
			3,
			{
				dryMassKg*(y*y+z*z), -dryMassKg*x*y, -dryMassKg*x*z,
				-dryMassKg*x*y, dryMassKg*(x*x+z*z), -dryMassKg * y * z,
				-dryMassKg*x*z, -dryMassKg*y*z, dryMassKg*(x*x+y*y)
			});

		x = propellantOffsetB.x;
		y = propellantOffsetB.y;
		z = propellantOffsetB.z;

		math::Matrix propellantShift(
			3,
			3,
			{
				propellantMassKg* (y * y + z * z), -propellantMassKg * x * y, -propellantMassKg * x * z,
				-propellantMassKg * x * y, propellantMassKg* (x * x + z * z), -propellantMassKg * y * z,
				-propellantMassKg * x * z, -propellantMassKg * y * z, propellantMassKg* (x * x + y * y)
			});

		math::Matrix inertiaTensorBody=dryInertiaTensorBody + dryShift + propellantShift;

		if (!inertiaTensorBody.isFinite())
		{
			throw std::overflow_error("inertiaTensorBody must be finite");
		}

		return inertiaTensorBody;
	}
}