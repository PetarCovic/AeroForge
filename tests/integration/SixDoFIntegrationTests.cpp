#include <gtest/gtest.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MassProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyQuaternionNormalization.h>
#include <dynamics/RigidBodyState.h>
#include <dynamics/SixDofDynamicsModel.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <simulation/RK4Integrator.h>
#include <simulation/State.h>

#include <cmath>

namespace aeroforge::integration::tests
{
	TEST(
		SixDoFIntegrationTests,
		TranslationAndRotationEvolveSimultaneously)
	{
		dynamics::ForceAccumulator forces;

		forces.addForce(
			math::Vector3(
				10.0,
				0.0,
				0.0));

		dynamics::MomentAccumulator moments;

		moments.addMoment(
			math::Vector3(
				0.0,
				0.0,
				2.0));

		dynamics::MassProperties mass(
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

		simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0))
			.toStateVector();

		constexpr double dt =
			0.001;

		for (int step = 0;
			step < 1000;
			++step)
		{
			state =
				integrator.step(
					model,
					step * dt,
					state,
					dt);

			dynamics::
				normalizeRigidBodyQuaternion(
					state);
		}

		EXPECT_GT(
			state[0],
			0.0);

		EXPECT_GT(
			state[3],
			0.0);

		EXPECT_GT(
			state[9],
			0.0);

		EXPECT_GT(
			state[12],
			0.0);
	}

	TEST(
		SixDoFIntegrationTests,
		QuaternionRemainsUnitLengthDuringLongRun)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		dynamics::MassProperties mass(
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

		simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.3,
					0.4,
					0.5))
			.toStateVector();

		constexpr double dt =
			0.001;

		for (int step = 0;
			step < 100000;
			++step)
		{
			state =
				integrator.step(
					model,
					step * dt,
					state,
					dt);

			dynamics::
				normalizeRigidBodyQuaternion(
					state);
		}

		const double quaternionNorm =
			std::sqrt(
				state[6] * state[6]
				+ state[7] * state[7]
				+ state[8] * state[8]
				+ state[9] * state[9]);

		EXPECT_NEAR(
			quaternionNorm,
			1.0,
			1e-12);

		EXPECT_TRUE(
			state.isFinite());
	}

	TEST(
		SixDoFIntegrationTests,
		ZeroForceZeroMomentRestStateRemainsAtRest)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		dynamics::MassProperties mass(
			10.0);

		const dynamics::InertiaProperties inertia(
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				}));

		const dynamics::SixDoFDynamicsModel model(
			forces,
			moments,
			mass,
			inertia);

		const simulation::RK4Integrator integrator;

		const simulation::State initialState =
			dynamics::RigidBodyState(
				math::Vector3(
					10.0,
					20.0,
					30.0),
				math::Vector3(
					0.0,
					0.0,
					0.0),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),
				math::Vector3(
					0.0,
					0.0,
					0.0))
			.toStateVector();

		simulation::State state =
			initialState;

		constexpr double dt =
			0.01;

		for (int step = 0;
			step < 1000;
			++step)
		{
			state =
				integrator.step(
					model,
					step * dt,
					state,
					dt);

			dynamics::
				normalizeRigidBodyQuaternion(
					state);
		}

		EXPECT_TRUE(
			state.isApprox(
				initialState,
				1e-12));
	}
}