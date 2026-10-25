#include <dynamics/RigidBodyState.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::dynamics
{
    RigidBodyState::RigidBodyState(
        math::Vector3 positionN,
        math::Vector3 velocityN,
        math::Quaternion attitudeNB,
        math::Vector3 angularVelocityB)
        : positionN(positionN), velocityN(velocityN), attitudeNB(attitudeNB), angularVelocityB(angularVelocityB)
    {
        if (!positionN.isFinite())
        {
            throw std::invalid_argument("positionN must be finite");
        }

        if (!velocityN.isFinite())
        {
            throw std::invalid_argument("velocityN must be finite");
        }

        if (!attitudeNB.isFinite() || attitudeNB.isZero())
        {
            throw std::invalid_argument("attitudeNB must be finite and non-zero");
        }

        if (!angularVelocityB.isFinite())
        {
            throw std::invalid_argument("angularVelocityB must be finite");
        }
    }

    RigidBodyState::RigidBodyState(
        const simulation::State& stateVector)
    {
        if (stateVector.size()!=STATE_SIZE)
        {
            throw std::invalid_argument("stateVector must have exactly 13 elements");
        }

        if (!stateVector.isFinite())
        {
            throw std::invalid_argument("stateVector must be finite");
        }

        positionN.x = stateVector[0];
        positionN.y = stateVector[1];
        positionN.z = stateVector[2];

        velocityN.x = stateVector[3];
        velocityN.y = stateVector[4];
        velocityN.z = stateVector[5];

        attitudeNB.w = stateVector[6];
        attitudeNB.x = stateVector[7];
        attitudeNB.y = stateVector[8];
        attitudeNB.z = stateVector[9];

        if (!attitudeNB.isFinite() || attitudeNB.isZero())
        {
            throw std::invalid_argument("attitudeNB must be finite and non-zero");
        }

        angularVelocityB.x = stateVector[10];
        angularVelocityB.y = stateVector[11];
        angularVelocityB.z = stateVector[12];
    }

    const math::Vector3& RigidBodyState::getPositionN() const
    {
        return positionN;
    }

    const math::Vector3& RigidBodyState::getVelocityN() const
    {
        return velocityN;
    }

    const math::Quaternion& RigidBodyState::getAttitudeNB() const
    {
        return attitudeNB;
    }

    const math::Vector3& RigidBodyState::getAngularVelocityB() const
    {
        return angularVelocityB;
    }

    simulation::State RigidBodyState::toStateVector() const
    {
        return simulation::State({ 
            positionN.x,
            positionN.y,
            positionN.z,
            velocityN.x,
            velocityN.y,
            velocityN.z,
            attitudeNB.w,
            attitudeNB.x,
            attitudeNB.y,
            attitudeNB.z,
            angularVelocityB.x,
            angularVelocityB.y,
            angularVelocityB.z 
            });
    }
}