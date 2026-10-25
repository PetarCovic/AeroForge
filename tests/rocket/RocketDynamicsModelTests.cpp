#include <gtest/gtest.h>

#include <rocket/PropellantTank.h>
#include <rocket/RocketDynamicsModel.h>
#include <rocket/RocketEngine.h>
#include <rocket/RocketMassProperties.h>
#include <rocket/ThrustCurve.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyState.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>
#include <vector>

namespace aeroforge::rocket::tests
{
	namespace
	{
		ThrustCurve makeConstantCurve(
			double thrustN)
		{
			return ThrustCurve(
				std::vector<ThrustPoint>{
					{0.0, thrustN},
					{ 10.0, thrustN }
			});
		}

		RocketMassProperties makeMassProperties()
		{
			return RocketMassProperties(
				9.0,
				math::Vector3(0.0, 0.0, 0.0),
				math::Matrix(
					3,
					3,
					{
						2.0, 0.0, 0.0,
						0.0, 2.0, 0.0,
						0.0, 0.0, 2.0
					}),
				math::Vector3(0.0, 0.0, 0.0));
		}
	}

	TEST(
		RocketDynamicsModelTests,
		ReturnsThirteenElementDerivative)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		PropellantTank tank(
			1.0,
			1.0);

		const RocketMassProperties massProperties =
			makeMassProperties();

		const RocketEngine engine(
			makeConstantCurve(0.0),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const RocketDynamicsModel model(
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

		EXPECT_EQ(
			model.evaluate(
				0.0,
				state).size(),
			13);
	}

	TEST(
		RocketDynamicsModelTests,
		BodyXThrustAtIdentityProducesNorthAcceleration)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		PropellantTank tank(
			1.0,
			1.0);

		const RocketEngine engine(
			makeConstantCurve(20.0),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const RocketDynamicsModel model(
			forces,
			moments,
			tank,
			makeMassProperties(),
			engine);

		const simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

		const simulation::StateDerivative derivative =
			model.evaluate(
				1.0,
				state);

		EXPECT_NEAR(
			derivative[3],
			2.0,
			1e-12);

		EXPECT_NEAR(
			derivative[4],
			0.0,
			1e-12);

		EXPECT_NEAR(
			derivative[5],
			0.0,
			1e-12);
	}

	TEST(
		RocketDynamicsModelTests,
		AttitudeTransformsBodyThrustIntoNavigationFrame)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		PropellantTank tank(
			1.0,
			1.0);

		const RocketEngine engine(
			makeConstantCurve(20.0),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const RocketDynamicsModel model(
			forces,
			moments,
			tank,
			makeMassProperties(),
			engine);

		const double rootHalf =
			std::sqrt(0.5);

		const simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(
					rootHalf,
					0.0,
					0.0,
					rootHalf),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

		const simulation::StateDerivative derivative =
			model.evaluate(
				1.0,
				state);

		EXPECT_NEAR(
			derivative[3],
			0.0,
			1e-10);

		EXPECT_NEAR(
			derivative[4],
			2.0,
			1e-10);
	}

	TEST(
		RocketDynamicsModelTests,
		OffsetEngineProducesAngularAcceleration)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		PropellantTank tank(
			1.0,
			1.0);

		const RocketEngine engine(
			makeConstantCurve(10.0),
			200.0,
			math::Vector3(0.0, 1.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const RocketDynamicsModel model(
			forces,
			moments,
			tank,
			makeMassProperties(),
			engine);

		const simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

		const simulation::StateDerivative derivative =
			model.evaluate(
				1.0,
				state);

		EXPECT_NEAR(
			derivative[12],
			-5.0,
			1e-12);
	}

	TEST(
		RocketDynamicsModelTests,
		EmptyTankProducesNoEngineThrust)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		PropellantTank tank(
			1.0,
			0.0);

		const RocketEngine engine(
			makeConstantCurve(100.0),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const RocketDynamicsModel model(
			forces,
			moments,
			tank,
			makeMassProperties(),
			engine);

		const simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

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
		RocketDynamicsModelTests,
		EvaluateDoesNotConsumePropellant)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		PropellantTank tank(
			1.0,
			1.0);

		const RocketEngine engine(
			makeConstantCurve(100.0),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const RocketDynamicsModel model(
			forces,
			moments,
			tank,
			makeMassProperties(),
			engine);

		const simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

		model.evaluate(
			1.0,
			state);

		model.evaluate(
			1.0,
			state);

		model.evaluate(
			1.0,
			state);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantMassKg(),
			1.0);
	}

	TEST(
		RocketDynamicsModelTests,
		UsesTimeDependentThrustCurve)
	{
		dynamics::ForceAccumulator forces;
		dynamics::MomentAccumulator moments;

		PropellantTank tank(
			1.0,
			1.0);

		const RocketEngine engine(
			ThrustCurve(
				std::vector<ThrustPoint>{
					{0.0, 0.0},
					{ 1.0, 20.0 }
		}),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const RocketDynamicsModel model(
			forces,
			moments,
			tank,
			makeMassProperties(),
			engine);

		const simulation::State state =
			dynamics::RigidBodyState(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))
			.toStateVector();

		const simulation::StateDerivative derivative =
			model.evaluate(
				0.5,
				state);

		EXPECT_NEAR(
			derivative[3],
			1.0,
			1e-12);
	}
}