#pragma once

#include <math/Vector3.h>

namespace aeroforge::dynamics
{
	class MomentAccumulator
	{
	public:
		MomentAccumulator();

		void addMoment(const math::Vector3& momentB);

		const math::Vector3& getTotalMomentB() const;

		void clear();

	private:
		math::Vector3 totalMomentB;
	};
}