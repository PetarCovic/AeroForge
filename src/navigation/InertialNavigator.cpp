#include <navigation/InertialNavigator.h>
#include <dynamics/RotationalKinematics.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::navigation
{
	InertialNavigator::InertialNavigator()
	{
	}

	NavigationState InertialNavigator::propagate(
		const NavigationState& currentState,
		const sensors::ImuMeasurement& imuMeasurement,
		const math::Vector3& gravityN,
		double dtS
	) const
	{
		if (!std::isfinite(dtS) || dtS < 0.0)
		{
			throw std::invalid_argument(
				"dtS must be finite and non-negative");
		}

		if (!std::isfinite(imuMeasurement.timeS) ||
			imuMeasurement.timeS < 0.0)
		{
			throw std::invalid_argument(
				"IMU measurement time must be finite and non-negative");
		}

		if (!imuMeasurement.specificForceB.isFinite())
		{
			throw std::invalid_argument(
				"IMU specific force must be finite");
		}

		if (!imuMeasurement.angularVelocityB.isFinite())
		{
			throw std::invalid_argument(
				"IMU angular velocity must be finite");
		}

		if (!gravityN.isFinite())
		{
			throw std::invalid_argument(
				"gravityN must be finite");
		}


		const math::Vector3& currentPositionN =
			currentState.getPositionN();

		const math::Vector3& currentVelocityN =
			currentState.getVelocityN();

		const math::Quaternion& currentAttitudeNB =
			currentState.getAttitudeNB();

		math::Vector3 specificForceN =
			currentAttitudeNB.rotate(
				imuMeasurement.specificForceB
			);

		math::Vector3 accelerationN =
			specificForceN + gravityN;

		math::Vector3 nextPositionN =
			currentPositionN
			+ currentVelocityN * dtS
			+ accelerationN * (0.5 * dtS * dtS);

		math::Vector3 nextVelocityN =
			currentVelocityN
			+ accelerationN * dtS;


		// qDot_NB = 0.5 * q_NB * [0, omega_B]
		math::Quaternion quaternionDerivative =
			dynamics::computeQuaternionDerivativeNB(
				currentAttitudeNB,
				imuMeasurement.angularVelocityB
			);

		math::Quaternion nextAttitudeNB(
			currentAttitudeNB.w
			+ quaternionDerivative.w * dtS,

			currentAttitudeNB.x
			+ quaternionDerivative.x * dtS,

			currentAttitudeNB.y
			+ quaternionDerivative.y * dtS,

			currentAttitudeNB.z
			+ quaternionDerivative.z * dtS
		);

		nextAttitudeNB =
			nextAttitudeNB.normalized();


		if (!nextPositionN.isFinite() ||
			!nextVelocityN.isFinite() ||
			!nextAttitudeNB.isFinite())
		{
			throw std::overflow_error(
				"Inertial navigation propagation produced a non-finite state");
		}


		return NavigationState{
			nextPositionN,
			nextVelocityN,
			nextAttitudeNB,
			imuMeasurement.angularVelocityB
		};
	}
}