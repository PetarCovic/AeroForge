#include <gtest/gtest.h>

#include <rocket/PropellantDepletion.h>
#include <rocket/PropellantTank.h>
#include <rocket/RocketDynamicsModel.h>
#include <rocket/RocketEngine.h>
#include <rocket/RocketMassProperties.h>
#include <rocket/ThrustCurve.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyQuaternionNormalization.h>
#include <dynamics/RigidBodyState.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <simulation/RK4Integrator.h>
#include <simulation/State.h>

#include <vector>

namespace aeroforge::integration::tests
{
	TEST(
		RocketIntegrationTests,
		PropellantMassDecreasesOncePerCompletedStep)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		rocket::PropellantTank tank(
			1.0,
			1.0);

		constexpr double isp =
			100.0;

		constexpr double thrust =
			isp * 9.80665;

		const rocket::RocketEngine engine(
			rocket::ThrustCurve(
				std::vector<rocket::ThrustPoint>{
					{0.0, thrust},
					{ 10.0, thrust }
		}),
			isp,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const rocket::RocketMassProperties massProperties(
			9.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix::diagonal(
				2.0,
				3),
			math::Vector3(0.0, 0.0, 0.0));

		const rocket::RocketDynamicsModel model(
			forces,
			moments,
			tank,
			massProperties,
			engine);

		const simulation::RK4Integrator integrator;

		simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

		constexpr double dt =
			0.1;

		for (int step = 0;
			step < 5;
			++step)
		{
			const double time =
				step * dt;

			state =
				integrator.step(
					model,
					time,
					state,
					dt);

			dynamics::
				normalizeRigidBodyQuaternion(
					state);

			rocket::
				consumePropellantForStep(
					tank,
					engine,
					time,
					dt,
					1.0);
		}

		EXPECT_NEAR(
			tank.getPropellantMassKg(),
			0.5,
			1e-12);
	}

	TEST(
		RocketIntegrationTests,
		EngineStopsProducingThrustAfterPropellantDepletion)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		rocket::PropellantTank tank(
			1.0,
			0.1);

		constexpr double isp =
			100.0;

		constexpr double thrust =
			isp * 9.80665;

		const rocket::RocketEngine engine(
			rocket::ThrustCurve(
				std::vector<rocket::ThrustPoint>{
					{0.0, thrust},
					{ 10.0, thrust }
		}),
			isp,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const rocket::RocketMassProperties massProperties(
			9.9,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix::diagonal(
				2.0,
				3),
			math::Vector3(0.0, 0.0, 0.0));

		const rocket::RocketDynamicsModel model(
			forces,
			moments,
			tank,
			massProperties,
			engine);

		const simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

		rocket::consumePropellantForStep(
			tank,
			engine,
			0.0,
			1.0,
			1.0);

		ASSERT_TRUE(
			tank.isEmpty());

		const simulation::StateDerivative derivative =
			model.evaluate(
				1.0,
				state);

		EXPECT_NEAR(
			derivative[3],
			0.0,
			1e-12);
	}

	TEST(
		RocketIntegrationTests,
		OffAxisEngineProducesTranslationAndRotation)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		rocket::PropellantTank tank(
			1.0,
			1.0);

		const rocket::RocketEngine engine(
			rocket::ThrustCurve(
				std::vector<rocket::ThrustPoint>{
					{0.0, 10.0},
					{ 10.0, 10.0 }
		}),
			200.0,
			math::Vector3(0.0, 1.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const rocket::RocketMassProperties massProperties(
			9.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix::diagonal(
				2.0,
				3),
			math::Vector3(0.0, 0.0, 0.0));

		const rocket::RocketDynamicsModel model(
			forces,
			moments,
			tank,
			massProperties,
			engine);

		const simulation::RK4Integrator integrator;

		simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
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

		EXPECT_LT(
			state[9],
			0.0);

		EXPECT_LT(
			state[12],
			0.0);
	}

	TEST(
		RocketIntegrationTests,
		ChangingPropellantChangesMassCenterOfMassAndInertia)
	{
		const rocket::RocketMassProperties massProperties(
			8.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				}),
			math::Vector3(1.0, 0.0, 0.0));

		const double fullMass =
			massProperties.getTotalMassKg(
				2.0);

		const double halfMass =
			massProperties.getTotalMassKg(
				1.0);

		const math::Vector3 fullCenter =
			massProperties.getCenterOfMassB(
				2.0);

		const math::Vector3 halfCenter =
			massProperties.getCenterOfMassB(
				1.0);

		const math::Matrix fullInertia =
			massProperties.getInertiaTensorBody(
				2.0);

		const math::Matrix halfInertia =
			massProperties.getInertiaTensorBody(
				1.0);

		EXPECT_GT(
			fullMass,
			halfMass);

		EXPECT_GT(
			fullCenter.x,
			halfCenter.x);

		EXPECT_GT(
			fullInertia(1, 1),
			halfInertia(1, 1));
	}
}