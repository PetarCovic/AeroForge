#include <gtest/gtest.h>

#include <actuators/BodyMomentActuator.h>

#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeSetpoint.h>
#include <gnc/HoldAttitudeGuidance.h>

#include <navigation/NavigationState.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MassProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyQuaternionNormalization.h>
#include <dynamics/RigidBodyState.h>
#include <dynamics/SixDoFDynamicsModel.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <simulation/RK4Integrator.h>
#include <simulation/State.h>

#include <cmath>

namespace aeroforge::integration::tests
{
	TEST(
		GncClosedLoopIntegrationTests,
		AttitudeControllerConvergesFromInitialYawError)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		const dynamics::MassProperties mass(
			10.0);

		const dynamics::InertiaProperties inertia(
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 2.0, 0.0,
					0.0, 0.0, 2.0
				}));

		const dynamics::SixDoFDynamicsModel model(
			forces,
			moments,
			mass,
			inertia);

		const simulation::RK4Integrator integrator;

		const gnc::AttitudeSetpoint target(
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const gnc::HoldAttitudeGuidance guidance(
			target);

		const gnc::AttitudePdController controller(
			math::Vector3(
				8.0,
				8.0,
				8.0),
			math::Vector3(
				5.0,
				5.0,
				5.0),
			math::Vector3(
				20.0,
				20.0,
				20.0));

		actuators::BodyMomentActuator actuator(
			math::Vector3(
				20.0,
				20.0,
				20.0),
			math::Vector3(
				200.0,
				200.0,
				200.0));

		const double initialYaw =
			20.0
			* 3.14159265358979323846
			/ 180.0;

		simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					std::cos(
						initialYaw / 2.0),
					0.0,
					0.0,
					std::sin(
						initialYaw / 2.0)),
				math::Vector3())
			.toStateVector();

		constexpr double dt =
			0.005;

		constexpr int steps =
			1600;

		for (int step = 0;
			step < steps;
			++step)
		{
			const double time =
				step * dt;

			const dynamics::RigidBodyState truth(
				state);

			const navigation::NavigationState nav(
				truth.getPositionN(),
				truth.getVelocityN(),
				truth.getAttitudeNB(),
				truth.getAngularVelocityB());

			const gnc::AttitudeSetpoint setpoint =
				guidance.getSetpoint(
					time);

			const math::Vector3 command =
				controller.computeMomentCommandB(
					setpoint,
					nav);

			const math::Vector3 actual =
				actuator.update(
					command,
					dt);

			moments.clear();
			moments.addMoment(
				actual);

			state =
				integrator.step(
					model,
					time,
					state,
					dt);

			dynamics::
				normalizeRigidBodyQuaternion(
					state);
		}

		const dynamics::RigidBodyState finalState(
			state);

		const math::Quaternion& finalAttitude =
			finalState.getAttitudeNB();

		const double finalAngleError =
			2.0
			* std::atan2(
				std::abs(finalAttitude.z),
				std::abs(finalAttitude.w));

		EXPECT_LT(
			finalAngleError,
			1.0
			* 3.14159265358979323846
			/ 180.0);

		EXPECT_LT(
			std::abs(
				finalState
				.getAngularVelocityB()
				.z),
			0.05);
	}
}