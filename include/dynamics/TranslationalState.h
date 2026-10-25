#pragma once

#include <math/Vector3.h>
#include <simulation/State.h>

#include <cstddef>

namespace aeroforge::dynamics {
	class TranslationalState{
		//0 = position North
		//1 = position East
		//2 = position Down
		//3 = velocity North
		//4 = velocity East
		//5 = velocity Down

	public:
		static constexpr std::size_t STATE_SIZE = 6;

		TranslationalState(aeroforge::math::Vector3 position = aeroforge::math::Vector3(0.0, 0.0, 0.0),
			aeroforge::math::Vector3 velocity = aeroforge::math::Vector3(0.0, 0.0, 0.0));

		TranslationalState(const aeroforge::simulation::State& stateVector);

		const aeroforge::math::Vector3& getPosition() const;

		const aeroforge::math::Vector3& getVelocity() const;

		aeroforge::simulation::State toStateVector() const;

	private:
		aeroforge::math::Vector3 positionN;

		aeroforge::math::Vector3 velocityN;
	};
} // namespace aeroforge::dynamics