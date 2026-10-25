#include <dynamics/SixDoFDynamicsModel.h>
#include <dynamics/RigidBodyState.h>
#include <dynamics/RotationalKinematics.h>
#include <dynamics/RotationalDynamics.h>

#include <stdexcept>

namespace aeroforge::dynamics
{
	SixDoFDynamicsModel::SixDoFDynamicsModel(
		const ForceAccumulator& forceAccumulator,
		const MomentAccumulator& momentAccumulator,
		const MassProperties& massProperties,
		const InertiaProperties& inertiaProperties
	)
		: forceAccumulator(forceAccumulator), momentAccumulator(momentAccumulator), massProperties(massProperties), inertiaProperties(inertiaProperties)
	{
	}

	simulation::StateDerivative SixDoFDynamicsModel::evaluate(
		double time,
		const simulation::State& state) const
	{
		(void)time;

		RigidBodyState rigidBodyState(state);

		const math::Vector3& velocityN =
			rigidBodyState.getVelocityN();

		math::Vector3 linearAccelerationN =
			forceAccumulator.getTotalForce()
			/ massProperties.getMassKg();

		math::Quaternion quaternionDerivative =
			computeQuaternionDerivativeNB(
				rigidBodyState.getAttitudeNB(),
				rigidBodyState.getAngularVelocityB());

		math::Vector3 angularAccelerationB =
			computeAngularAccelerationBody(
				inertiaProperties,
				momentAccumulator.getTotalMomentB(),
				rigidBodyState.getAngularVelocityB());

		simulation::StateDerivative derivative({
			velocityN.x,
			velocityN.y,
			velocityN.z,

			linearAccelerationN.x,
			linearAccelerationN.y,
			linearAccelerationN.z,

			quaternionDerivative.w,
			quaternionDerivative.x,
			quaternionDerivative.y,
			quaternionDerivative.z,

			angularAccelerationB.x,
			angularAccelerationB.y,
			angularAccelerationB.z
			});

		if (!derivative.isFinite())
		{
			throw std::overflow_error(
				"SixDoF state derivative must be finite");
		}

		return derivative;
	}
}