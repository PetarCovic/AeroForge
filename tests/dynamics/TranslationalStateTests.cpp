#include <gtest/gtest.h>

#include <dynamics/TranslationalState.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		TranslationalStateTests,
		DefaultConstructionCreatesZeroPositionAndVelocity)
	{
		const TranslationalState state;

		EXPECT_DOUBLE_EQ(
			state.getPosition().x,
			0.0);

		EXPECT_DOUBLE_EQ(
			state.getPosition().y,
			0.0);

		EXPECT_DOUBLE_EQ(
			state.getPosition().z,
			0.0);

		EXPECT_DOUBLE_EQ(
			state.getVelocity().x,
			0.0);

		EXPECT_DOUBLE_EQ(
			state.getVelocity().y,
			0.0);

		EXPECT_DOUBLE_EQ(
			state.getVelocity().z,
			0.0);
	}

	TEST(
		TranslationalStateTests,
		ConstructsFromPositionAndVelocity)
	{
		const math::Vector3 position(
			100.0,
			50.0,
			-25.0);

		const math::Vector3 velocity(
			10.0,
			-5.0,
			2.0);

		const TranslationalState state(
			position,
			velocity);

		EXPECT_TRUE(
			state.getPosition().isApprox(
				position,
				1e-12));

		EXPECT_TRUE(
			state.getVelocity().isApprox(
				velocity,
				1e-12));
	}

	TEST(
		TranslationalStateTests,
		StateSizeConstantIsSix)
	{
		EXPECT_EQ(
			TranslationalState::STATE_SIZE,
			6U);
	}

	TEST(
		TranslationalStateTests,
		ConstructsFromGenericStateUsingCorrectNEDMapping)
	{
		const simulation::State rawState{
			100.0,
			200.0,
			-300.0,
			10.0,
			20.0,
			-30.0
		};

		const TranslationalState state(
			rawState);

		EXPECT_DOUBLE_EQ(
			state.getPosition().x,
			100.0);

		EXPECT_DOUBLE_EQ(
			state.getPosition().y,
			200.0);

		EXPECT_DOUBLE_EQ(
			state.getPosition().z,
			-300.0);

		EXPECT_DOUBLE_EQ(
			state.getVelocity().x,
			10.0);

		EXPECT_DOUBLE_EQ(
			state.getVelocity().y,
			20.0);

		EXPECT_DOUBLE_EQ(
			state.getVelocity().z,
			-30.0);
	}

	TEST(
		TranslationalStateTests,
		ToStateVectorUsesCorrectNEDMapping)
	{
		const TranslationalState state(
			math::Vector3(
				1.0,
				2.0,
				3.0),
			math::Vector3(
				4.0,
				5.0,
				6.0));

		const simulation::State rawState =
			state.toStateVector();

		ASSERT_EQ(
			rawState.size(),
			TranslationalState::STATE_SIZE);

		EXPECT_DOUBLE_EQ(
			rawState[0],
			1.0);

		EXPECT_DOUBLE_EQ(
			rawState[1],
			2.0);

		EXPECT_DOUBLE_EQ(
			rawState[2],
			3.0);

		EXPECT_DOUBLE_EQ(
			rawState[3],
			4.0);

		EXPECT_DOUBLE_EQ(
			rawState[4],
			5.0);

		EXPECT_DOUBLE_EQ(
			rawState[5],
			6.0);
	}

	TEST(
		TranslationalStateTests,
		GenericStateRoundTripPreservesValues)
	{
		const simulation::State original{
			123.4,
			-56.7,
			890.1,
			12.3,
			-45.6,
			78.9
		};

		const TranslationalState state(
			original);

		const simulation::State result =
			state.toStateVector();

		EXPECT_TRUE(
			result.isApprox(
				original,
				1e-12));
	}

	TEST(
		TranslationalStateTests,
		VectorRoundTripPreservesPositionAndVelocity)
	{
		const math::Vector3 originalPosition(
			12.0,
			-34.0,
			56.0);

		const math::Vector3 originalVelocity(
			-7.0,
			8.0,
			-9.0);

		const TranslationalState original(
			originalPosition,
			originalVelocity);

		const TranslationalState reconstructed(
			original.toStateVector());

		EXPECT_TRUE(
			reconstructed.getPosition()
			.isApprox(
				originalPosition,
				1e-12));

		EXPECT_TRUE(
			reconstructed.getVelocity()
			.isApprox(
				originalVelocity,
				1e-12));
	}

	TEST(
		TranslationalStateTests,
		RejectsGenericStateWithTooFewElements)
	{
		const simulation::State rawState{
			1.0,
			2.0,
			3.0,
			4.0,
			5.0
		};

		EXPECT_THROW(
			TranslationalState{ rawState },
			std::invalid_argument);
	}

	TEST(
		TranslationalStateTests,
		RejectsGenericStateWithTooManyElements)
	{
		const simulation::State rawState{
			1.0,
			2.0,
			3.0,
			4.0,
			5.0,
			6.0,
			7.0
		};

		EXPECT_THROW(
			TranslationalState{ rawState },
			std::invalid_argument);
	}

	TEST(
		TranslationalStateTests,
		RejectsNaNPosition)
	{
		const math::Vector3 position(
			0.0,
			std::numeric_limits<double>::quiet_NaN(),
			0.0);

		const math::Vector3 velocity(
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			TranslationalState(
				position,
				velocity),
			std::invalid_argument);
	}

	TEST(
		TranslationalStateTests,
		RejectsInfinitePosition)
	{
		const math::Vector3 position(
			0.0,
			std::numeric_limits<double>::infinity(),
			0.0);

		const math::Vector3 velocity(
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			TranslationalState(
				position,
				velocity),
			std::invalid_argument);
	}

	TEST(
		TranslationalStateTests,
		RejectsNaNVelocity)
	{
		const math::Vector3 position(
			0.0,
			0.0,
			0.0);

		const math::Vector3 velocity(
			0.0,
			std::numeric_limits<double>::quiet_NaN(),
			0.0);

		EXPECT_THROW(
			TranslationalState(
				position,
				velocity),
			std::invalid_argument);
	}

	TEST(
		TranslationalStateTests,
		RejectsInfiniteVelocity)
	{
		const math::Vector3 position(
			0.0,
			0.0,
			0.0);

		const math::Vector3 velocity(
			0.0,
			0.0,
			-std::numeric_limits<double>::infinity());

		EXPECT_THROW(
			TranslationalState(
				position,
				velocity),
			std::invalid_argument);
	}

	TEST(
		TranslationalStateTests,
		RejectsNonFiniteGenericState)
	{
		const simulation::State rawState{
			1.0,
			2.0,
			3.0,
			4.0,
			std::numeric_limits<double>::quiet_NaN(),
			6.0
		};

		EXPECT_THROW(
			TranslationalState{ rawState },
			std::invalid_argument);
	}

	TEST(
		TranslationalStateTests,
		NegativeDownCoordinateRepresentsPositionAboveOrigin)
	{
		const TranslationalState state(
			math::Vector3(
				0.0,
				0.0,
				-100.0),
			math::Vector3(
				0.0,
				0.0,
				0.0));

		EXPECT_DOUBLE_EQ(
			state.getPosition().z,
			-100.0);

		const simulation::State rawState =
			state.toStateVector();

		EXPECT_DOUBLE_EQ(
			rawState[2],
			-100.0);
	}

	TEST(
		TranslationalStateTests,
		PositiveDownVelocityIsPreserved)
	{
		const TranslationalState state(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				25.0));

		EXPECT_DOUBLE_EQ(
			state.getVelocity().z,
			25.0);

		EXPECT_DOUBLE_EQ(
			state.toStateVector()[5],
			25.0);
	}

	TEST(
		TranslationalStateTests,
		GettersWorkThroughConstObject)
	{
		const TranslationalState state(
			math::Vector3(
				1.0,
				2.0,
				3.0),
			math::Vector3(
				4.0,
				5.0,
				6.0));

		const math::Vector3& position =
			state.getPosition();

		const math::Vector3& velocity =
			state.getVelocity();

		EXPECT_DOUBLE_EQ(
			position.x,
			1.0);

		EXPECT_DOUBLE_EQ(
			velocity.x,
			4.0);
	}
}