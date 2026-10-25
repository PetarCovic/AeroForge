#include <gtest/gtest.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/MassProperties.h>
#include <dynamics/TranslationalDynamicsModel.h>
#include <simulation/State.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		TranslationalDynamicsModelTests,
		ZeroForceProducesZeroAcceleration)
	{
		ForceAccumulator forces;
		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative derivative =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(derivative[3], 0.0);
		EXPECT_DOUBLE_EQ(derivative[4], 0.0);
		EXPECT_DOUBLE_EQ(derivative[5], 0.0);
	}

	TEST(
		TranslationalDynamicsModelTests,
		PositionDerivativeEqualsVelocity)
	{
		ForceAccumulator forces;
		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			100.0, 200.0, -50.0,
			12.0, -7.0, 3.5
		};

		const simulation::StateDerivative derivative =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(derivative[0], 12.0);
		EXPECT_DOUBLE_EQ(derivative[1], -7.0);
		EXPECT_DOUBLE_EQ(derivative[2], 3.5);
	}

	TEST(
		TranslationalDynamicsModelTests,
		SingleAxisForceProducesExpectedAcceleration)
	{
		ForceAccumulator forces;
		forces.addForce(
			math::Vector3(100.0, 0.0, 0.0));

		MassProperties mass(20.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative derivative =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(derivative[3], 5.0);
		EXPECT_DOUBLE_EQ(derivative[4], 0.0);
		EXPECT_DOUBLE_EQ(derivative[5], 0.0);
	}

	TEST(
		TranslationalDynamicsModelTests,
		MultiAxisForceProducesExpectedAcceleration)
	{
		ForceAccumulator forces;
		forces.addForce(
			math::Vector3(
				100.0,
				-50.0,
				200.0));

		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative derivative =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(derivative[3], 10.0);
		EXPECT_DOUBLE_EQ(derivative[4], -5.0);
		EXPECT_DOUBLE_EQ(derivative[5], 20.0);
	}

	TEST(
		TranslationalDynamicsModelTests,
		NegativeForceProducesNegativeAcceleration)
	{
		ForceAccumulator forces;
		forces.addForce(
			math::Vector3(
				-40.0,
				-20.0,
				-10.0));

		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative derivative =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(derivative[3], -4.0);
		EXPECT_DOUBLE_EQ(derivative[4], -2.0);
		EXPECT_DOUBLE_EQ(derivative[5], -1.0);
	}

	TEST(
		TranslationalDynamicsModelTests,
		PositiveDownForceProducesPositiveDownAcceleration)
	{
		ForceAccumulator forces;
		forces.addForce(
			math::Vector3(
				0.0,
				0.0,
				98.1));

		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative derivative =
			model.evaluate(0.0, state);

		EXPECT_NEAR(
			derivative[5],
			9.81,
			1e-12);
	}

	TEST(
		TranslationalDynamicsModelTests,
		NegativeDownForceProducesUpwardAcceleration)
	{
		ForceAccumulator forces;
		forces.addForce(
			math::Vector3(
				0.0,
				0.0,
				-50.0));

		MassProperties mass(5.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative derivative =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(
			derivative[5],
			-10.0);
	}

	TEST(
		TranslationalDynamicsModelTests,
		LargerMassProducesSmallerAcceleration)
	{
		ForceAccumulator forces;
		forces.addForce(
			math::Vector3(
				100.0,
				0.0,
				0.0));

		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative first =
			model.evaluate(0.0, state);

		mass.setMassKg(20.0);

		const simulation::StateDerivative second =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(first[3], 10.0);
		EXPECT_DOUBLE_EQ(second[3], 5.0);
	}

	TEST(
		TranslationalDynamicsModelTests,
		UsesUpdatedForceAccumulator)
	{
		ForceAccumulator forces;
		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		forces.addForce(
			math::Vector3(
				20.0,
				0.0,
				0.0));

		const simulation::StateDerivative first =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(
			first[3],
			2.0);

		forces.clear();

		forces.addForce(
			math::Vector3(
				50.0,
				0.0,
				0.0));

		const simulation::StateDerivative second =
			model.evaluate(0.0, state);

		EXPECT_DOUBLE_EQ(
			second[3],
			5.0);
	}

	TEST(
		TranslationalDynamicsModelTests,
		ReturnsSixElementDerivative)
	{
		ForceAccumulator forces;
		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			1.0, 2.0, 3.0,
			4.0, 5.0, 6.0
		};

		const simulation::StateDerivative derivative =
			model.evaluate(0.0, state);

		EXPECT_EQ(
			derivative.size(),
			TranslationalState::STATE_SIZE);
	}

	TEST(
		TranslationalDynamicsModelTests,
		RejectsStateWithIncorrectSize)
	{
		ForceAccumulator forces;
		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0,
			0.0,
			0.0
		};

		EXPECT_THROW(
			model.evaluate(
				0.0,
				state),
			std::invalid_argument);
	}

	TEST(
		TranslationalDynamicsModelTests,
		RejectsNonFiniteState)
	{
		ForceAccumulator forces;
		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0,
			0.0,
			0.0,
			std::numeric_limits<double>::quiet_NaN(),
			0.0,
			0.0
		};

		EXPECT_THROW(
			model.evaluate(
				0.0,
				state),
			std::invalid_argument);
	}

	TEST(
		TranslationalDynamicsModelTests,
		RejectsNonFiniteCalculatedAcceleration)
	{
		ForceAccumulator forces;

		forces.addForce(
			math::Vector3(
				std::numeric_limits<double>::max(),
				0.0,
				0.0));

		MassProperties mass(
			std::numeric_limits<double>::min());

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		EXPECT_THROW(
			model.evaluate(
				0.0,
				state),
			std::runtime_error);
	}

	TEST(
		TranslationalDynamicsModelTests,
		TimeDoesNotChangeCurrentDynamicsResult)
	{
		ForceAccumulator forces;

		forces.addForce(
			math::Vector3(
				10.0,
				20.0,
				30.0));

		MassProperties mass(10.0);

		TranslationalDynamicsModel model(forces, mass);

		simulation::State state{
			1.0, 2.0, 3.0,
			4.0, 5.0, 6.0
		};

		const simulation::StateDerivative first =
			model.evaluate(
				0.0,
				state);

		const simulation::StateDerivative second =
			model.evaluate(
				100.0,
				state);

		EXPECT_TRUE(
			first.isApprox(
				second,
				1e-12));
	}
}