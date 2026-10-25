#include <rocket/RocketDynamicsModel.h>
#include <rocket/ThrustMoment.h>

#include <dynamics/RigidBodyState.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/RotationalKinematics.h>
#include <dynamics/RotationalDynamics.h>

#include <stdexcept>

namespace aeroforge::rocket
{
	RocketDynamicsModel::RocketDynamicsModel(
		const dynamics::ForceAccumulator& externalForceAccumulator,
		const dynamics::MomentAccumulator& externalMomentAccumulator,
		const PropellantTank& propellantTank,
		const RocketMassProperties& rocketMassProperties,
		const RocketEngine& rocketEngine)
		: externalForceAccumulator(externalForceAccumulator),
		externalMomentAccumulator(externalMomentAccumulator),
		propellantTank(propellantTank),
		rocketMassProperties(rocketMassProperties),
		rocketEngine(rocketEngine)
	{
	}

	simulation::StateDerivative RocketDynamicsModel::evaluate(
		double time,
		const simulation::State& state) const
	{
		dynamics::RigidBodyState rigidBodyState(state);

		const double propellantMassKg =
			propellantTank.getPropellantMassKg();

		const double totalMassKg =
			rocketMassProperties.getTotalMassKg(
				propellantMassKg);

		const math::Vector3 centerOfMassB =
			rocketMassProperties.getCenterOfMassB(
				propellantMassKg);

		const math::Matrix inertiaTensorBody =
			rocketMassProperties.getInertiaTensorBody(
				propellantMassKg);

		const dynamics::InertiaProperties inertiaProperties(
			inertiaTensorBody);

		math::Vector3 thrustForceB;

		if (!propellantTank.isEmpty())
		{
			thrustForceB =
				rocketEngine.computeThrustForceB(
					time,
					1.0);
		}

		const math::Vector3 thrustForceN =
			rigidBodyState
			.getAttitudeNB()
			.rotate(thrustForceB);

		const math::Vector3 totalForceN =
			externalForceAccumulator.getTotalForce()
			+ thrustForceN;

		if (!totalForceN.isFinite())
		{
			throw std::overflow_error(
				"totalForceN must be finite");
		}

		const math::Vector3 linearAccelerationN =
			totalForceN / totalMassKg;

		if (!linearAccelerationN.isFinite())
		{
			throw std::overflow_error(
				"linearAccelerationN must be finite");
		}

		const math::Vector3 thrustMomentB =
			computeThrustMomentB(
				rocketEngine.getEnginePositionB(),
				centerOfMassB,
				thrustForceB);

		const math::Vector3 totalMomentB =
			externalMomentAccumulator.getTotalMomentB()
			+ thrustMomentB;

		if (!totalMomentB.isFinite())
		{
			throw std::overflow_error(
				"totalMomentB must be finite");
		}

		const math::Quaternion quaternionDerivative =
			dynamics::computeQuaternionDerivativeNB(
				rigidBodyState.getAttitudeNB(),
				rigidBodyState.getAngularVelocityB());

		const math::Vector3 angularAccelerationB =
			dynamics::computeAngularAccelerationBody(
				inertiaProperties,
				totalMomentB,
				rigidBodyState.getAngularVelocityB());

		const math::Vector3& velocityN =
			rigidBodyState.getVelocityN();

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
				"Rocket state derivative must be finite");
		}

		return derivative;
	}
}