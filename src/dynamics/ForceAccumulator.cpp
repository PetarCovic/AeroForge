#include <dynamics/ForceAccumulator.h>

#include <stdexcept>

namespace aeroforge::dynamics {
	ForceAccumulator::ForceAccumulator()
		: totalForceN(0.0, 0.0, 0.0)
	{
	}
	void ForceAccumulator::addForce(const aeroforge::math::Vector3& force)
	{
		if (!force.isFinite())
		{
			throw std::invalid_argument("Force must be finite.");
		}

		const aeroforge::math::Vector3 candidateTotalForce = totalForceN + force;

		if (!candidateTotalForce.isFinite())
		{
			throw std::overflow_error("Total force exceeded finite limits.");
		}

		totalForceN = candidateTotalForce;
	}
	const aeroforge::math::Vector3& ForceAccumulator::getTotalForce() const
	{
		return totalForceN;
	}
	void ForceAccumulator::clear()
	{
		totalForceN = aeroforge::math::Vector3(0.0, 0.0, 0.0);
	}
} // namespace aeroforge::dynamics