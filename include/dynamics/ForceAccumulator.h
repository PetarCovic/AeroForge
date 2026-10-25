#pragma once

#include <math/Vector3.h>

namespace aeroforge::dynamics {
	class ForceAccumulator {
	public:
		ForceAccumulator();

		void addForce(const aeroforge::math::Vector3& force);

		const aeroforge::math::Vector3& getTotalForce() const;

		void clear();

	private:
		aeroforge::math::Vector3 totalForceN;
	};
} // namespace aeroforge::dynamics