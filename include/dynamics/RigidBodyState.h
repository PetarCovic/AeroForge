#pragma once

#include <math/Vector3.h>
#include <math/Quaternion.h>
#include <simulation/State.h>

#include <cstddef>

namespace aeroforge::dynamics
{
	class RigidBodyState
	{
    public:
        static constexpr std::size_t STATE_SIZE = 13;

        RigidBodyState(
            math::Vector3 positionN,
            math::Vector3 velocityN,
            math::Quaternion attitudeNB,
            math::Vector3 angularVelocityB);

        explicit RigidBodyState(
            const simulation::State& stateVector);

        const math::Vector3& getPositionN() const;

        const math::Vector3& getVelocityN() const;

        const math::Quaternion& getAttitudeNB() const;

        const math::Vector3& getAngularVelocityB() const;

        simulation::State toStateVector() const;

    private:
        math::Vector3 positionN;
        math::Vector3 velocityN;
        math::Quaternion attitudeNB;
        math::Vector3 angularVelocityB;
	};
}