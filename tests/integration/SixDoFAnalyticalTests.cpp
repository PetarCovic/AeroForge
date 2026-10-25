#include <gtest/gtest.h>

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
		SixDoFAnalyticalTests,
		ConstantLinearForceMatchesAnalyticalMotion)
	{
		dynamics::ForceAccumulator forces;

		forces.addForce(
			math::Vector3(
				20.0,
				0.0,
				0.0));

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
					0.0,
					0.0,
					0.0))
			.toStateVector();

		constexpr double dt =
			0.01;

		for (int step = 0;
			step < 100;
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

		EXPECT_NEAR(
			state[0],
			1.0,
			1e-9);

		EXPECT_NEAR(
			state[3],
			2.0,
			1e-9);
	}

	TEST(
		SixDoFAnalyticalTests,
		ConstantSphericalBodySpinMatchesQuaternionSolution)
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

		constexpr double omega =
			1.0;

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
					omega))
			.toStateVector();

		constexpr double dt =
			0.001;

		constexpr int steps =
			1000;

		for (int step = 0;
			step < steps;
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

		const double expectedW =
			std::cos(0.5);

		const double expectedZ =
			std::sin(0.5);

		EXPECT_NEAR(
			state[6],
			expectedW,
			1e-8);

		EXPECT_NEAR(
			state[7],
			0.0,
			1e-8);

		EXPECT_NEAR(
			state[8],
			0.0,
			1e-8);

		EXPECT_NEAR(
			state[9],
			expectedZ,
			1e-8);

		EXPECT_NEAR(
			state[12],
			1.0,
			1e-10);
	}
}