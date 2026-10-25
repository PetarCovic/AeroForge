#include <dynamics/MomentAccumulator.h>

#include <stdexcept>

namespace aeroforge::dynamics
{
	MomentAccumulator::MomentAccumulator()
		: totalMomentB(0, 0, 0)
	{
	}

	void MomentAccumulator::addMoment(const math::Vector3& momentB)
	{
		if (!momentB.isFinite())
		{
			throw std::invalid_argument("momentB must be finite");
		}

		math::Vector3 candidateSum = totalMomentB+momentB;

		if (!candidateSum.isFinite())
		{
			throw std::overflow_error("candidateSum must be finite");
		}

		totalMomentB = candidateSum;
	}

	const math::Vector3& MomentAccumulator::getTotalMomentB() const
	{
		return totalMomentB;
	}

	void MomentAccumulator::clear()
	{
		totalMomentB.x = 0;
		totalMomentB.y = 0;
		totalMomentB.z = 0;
	}
}