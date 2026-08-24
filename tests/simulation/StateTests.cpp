#include <gtest/gtest.h>

#include <simulation/State.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <stdexcept>

namespace aeroforge::simulation::tests
{
	namespace
	{
		constexpr double EPSILON = 1e-10;

		void expectStateNear(
			const State& actual,
			const State& expected,
			double epsilon = EPSILON)
		{
			ASSERT_EQ(
				actual.size(),
				expected.size());

			for (std::size_t i = 0;
				i < actual.size();
				++i)
			{
				EXPECT_NEAR(
					actual[i],
					expected[i],
					epsilon);
			}
		}
	}

	// ============================================================
	// Construction
	// ============================================================

	TEST(StateConstructionTests, DefaultConstructorCreatesEmptyState)
	{
		const State state;

		EXPECT_EQ(
			state.size(),
			0U);

		EXPECT_TRUE(
			state.empty());
	}

	TEST(StateConstructionTests, SizedConstructorCreatesCorrectNumberOfElements)
	{
		const State state(5);

		EXPECT_EQ(
			state.size(),
			5U);

		EXPECT_FALSE(
			state.empty());
	}

	TEST(StateConstructionTests, SizedConstructorInitializesElementsToZero)
	{
		const State state(5);

		for (std::size_t i = 0;
			i < state.size();
			++i)
		{
			EXPECT_DOUBLE_EQ(
				state[i],
				0.0);
		}
	}

	TEST(StateConstructionTests, ZeroSizeCreatesEmptyState)
	{
		const State state(0);

		EXPECT_TRUE(
			state.empty());

		EXPECT_EQ(
			state.size(),
			0U);
	}

	TEST(StateConstructionTests, InitializerListStoresValues)
	{
		const State state{
			1.0,
			-2.0,
			3.5,
			4.25
		};

		ASSERT_EQ(
			state.size(),
			4U);

		EXPECT_DOUBLE_EQ(
			state[0],
			1.0);

		EXPECT_DOUBLE_EQ(
			state[1],
			-2.0);

		EXPECT_DOUBLE_EQ(
			state[2],
			3.5);

		EXPECT_DOUBLE_EQ(
			state[3],
			4.25);
	}

	// ============================================================
	// Size / Empty
	// ============================================================

	TEST(StatePropertyTests, SizeReturnsNumberOfElements)
	{
		const State state{
			1.0,
			2.0,
			3.0
		};

		EXPECT_EQ(
			state.size(),
			3U);
	}

	TEST(StatePropertyTests, EmptyReturnsFalseWhenStateContainsValues)
	{
		const State state{
			1.0
		};

		EXPECT_FALSE(
			state.empty());
	}

	// ============================================================
	// Indexing
	// ============================================================

	TEST(StateIndexTests, MutableIndexReturnsCorrectValue)
	{
		State state{
			1.0,
			2.0,
			3.0
		};

		state[1] = 10.0;

		EXPECT_DOUBLE_EQ(
			state[1],
			10.0);
	}

	TEST(StateIndexTests, ConstIndexReturnsCorrectValue)
	{
		const State state{
			1.0,
			2.0,
			3.0
		};

		EXPECT_DOUBLE_EQ(
			state[0],
			1.0);

		EXPECT_DOUBLE_EQ(
			state[1],
			2.0);

		EXPECT_DOUBLE_EQ(
			state[2],
			3.0);
	}

	TEST(StateIndexTests, MutableIndexRejectsOutOfRange)
	{
		State state{
			1.0,
			2.0
		};

		EXPECT_THROW(
			state[2],
			std::out_of_range);
	}

	TEST(StateIndexTests, ConstIndexRejectsOutOfRange)
	{
		const State state{
			1.0,
			2.0
		};

		EXPECT_THROW(
			state[100],
			std::out_of_range);
	}

	TEST(StateIndexTests, EmptyStateRejectsIndexZero)
	{
		State state;

		EXPECT_THROW(
			state[0],
			std::out_of_range);
	}

	// ============================================================
	// Addition
	// ============================================================

	TEST(StateAdditionTests, AdditionProducesCorrectResult)
	{
		const State a{
			1.0,
			2.0,
			3.0
		};

		const State b{
			4.0,
			5.0,
			6.0
		};

		const State result =
			a + b;

		expectStateNear(
			result,
			State{
				5.0,
				7.0,
				9.0
			});
	}

	TEST(StateAdditionTests, AdditionDoesNotModifyOperands)
	{
		const State a{
			1.0,
			2.0
		};

		const State b{
			3.0,
			4.0
		};

		const State result =
			a + b;

		(void)result;

		expectStateNear(
			a,
			State{
				1.0,
				2.0
			});

		expectStateNear(
			b,
			State{
				3.0,
				4.0
			});
	}

	TEST(StateAdditionTests, AdditionAssignmentProducesCorrectResult)
	{
		State a{
			1.0,
			2.0,
			3.0
		};

		const State b{
			4.0,
			5.0,
			6.0
		};

		a += b;

		expectStateNear(
			a,
			State{
				5.0,
				7.0,
				9.0
			});
	}

	TEST(StateAdditionTests, AdditionAssignmentReturnsReferenceToState)
	{
		State a{
			1.0,
			2.0
		};

		const State b{
			3.0,
			4.0
		};

		State& returned =
			(a += b);

		EXPECT_EQ(
			&returned,
			&a);
	}

	TEST(StateAdditionTests, AdditionRejectsMismatchedDimensions)
	{
		const State a{
			1.0,
			2.0
		};

		const State b{
			1.0,
			2.0,
			3.0
		};

		EXPECT_THROW(
			a + b,
			std::invalid_argument);
	}

	TEST(StateAdditionTests, AdditionAssignmentRejectsMismatchedDimensions)
	{
		State a{
			1.0,
			2.0
		};

		const State b{
			1.0,
			2.0,
			3.0
		};

		EXPECT_THROW(
			a += b,
			std::invalid_argument);
	}

	TEST(StateAdditionTests, EmptyStatesCanBeAdded)
	{
		const State a;
		const State b;

		const State result =
			a + b;

		EXPECT_TRUE(
			result.empty());
	}

	// ============================================================
	// Subtraction
	// ============================================================

	TEST(StateSubtractionTests, SubtractionProducesCorrectResult)
	{
		const State a{
			5.0,
			7.0,
			9.0
		};

		const State b{
			1.0,
			2.0,
			3.0
		};

		expectStateNear(
			a - b,
			State{
				4.0,
				5.0,
				6.0
			});
	}

	TEST(StateSubtractionTests, SubtractionAssignmentProducesCorrectResult)
	{
		State a{
			5.0,
			7.0,
			9.0
		};

		const State b{
			1.0,
			2.0,
			3.0
		};

		a -= b;

		expectStateNear(
			a,
			State{
				4.0,
				5.0,
				6.0
			});
	}

	TEST(StateSubtractionTests, SubtractionAssignmentReturnsReference)
	{
		State a{
			5.0,
			7.0
		};

		const State b{
			1.0,
			2.0
		};

		State& returned =
			(a -= b);

		EXPECT_EQ(
			&returned,
			&a);
	}

	TEST(StateSubtractionTests, SubtractionRejectsMismatchedDimensions)
	{
		const State a{
			1.0
		};

		const State b{
			1.0,
			2.0
		};

		EXPECT_THROW(
			a - b,
			std::invalid_argument);
	}

	TEST(StateSubtractionTests, SubtractionAssignmentRejectsMismatchedDimensions)
	{
		State a{
			1.0
		};

		const State b{
			1.0,
			2.0
		};

		EXPECT_THROW(
			a -= b,
			std::invalid_argument);
	}

	TEST(StateSubtractionTests, StateMinusItselfProducesZeroState)
	{
		const State state{
			1.5,
			-2.7,
			9.1,
			4.0
		};

		const State result =
			state - state;

		expectStateNear(
			result,
			State{
				0.0,
				0.0,
				0.0,
				0.0
			});
	}

	// ============================================================
	// Scalar multiplication
	// ============================================================

	TEST(StateScalarTests, StateTimesScalarProducesCorrectResult)
	{
		const State state{
			1.0,
			-2.0,
			3.0
		};

		expectStateNear(
			state * 2.5,
			State{
				2.5,
				-5.0,
				7.5
			});
	}

	TEST(StateScalarTests, ScalarTimesStateProducesCorrectResult)
	{
		const State state{
			1.0,
			-2.0,
			3.0
		};

		expectStateNear(
			2.5 * state,
			State{
				2.5,
				-5.0,
				7.5
			});
	}

	TEST(StateScalarTests, MultiplyAssignmentProducesCorrectResult)
	{
		State state{
			1.0,
			2.0,
			3.0
		};

		state *= 3.0;

		expectStateNear(
			state,
			State{
				3.0,
				6.0,
				9.0
			});
	}

	TEST(StateScalarTests, MultiplyAssignmentReturnsReference)
	{
		State state{
			1.0,
			2.0
		};

		State& returned =
			(state *= 2.0);

		EXPECT_EQ(
			&returned,
			&state);
	}

	TEST(StateScalarTests, MultiplicationByZeroProducesZeroState)
	{
		const State state{
			1.0,
			-7.0,
			4.0
		};

		expectStateNear(
			state * 0.0,
			State{
				0.0,
				0.0,
				0.0
			});
	}

	TEST(StateScalarTests, MultiplicationByOnePreservesState)
	{
		const State state{
			1.0,
			-7.0,
			4.0
		};

		expectStateNear(
			state * 1.0,
			state);
	}

	TEST(StateScalarTests, MultiplicationByNegativeScalarWorks)
	{
		const State state{
			1.0,
			-2.0
		};

		expectStateNear(
			state * -3.0,
			State{
				-3.0,
				6.0
			});
	}

	// ============================================================
	// Scalar division
	// ============================================================

	TEST(StateDivisionTests, DivisionProducesCorrectResult)
	{
		const State state{
			2.0,
			4.0,
			6.0
		};

		expectStateNear(
			state / 2.0,
			State{
				1.0,
				2.0,
				3.0
			});
	}

	TEST(StateDivisionTests, DivisionAssignmentProducesCorrectResult)
	{
		State state{
			2.0,
			4.0,
			6.0
		};

		state /= 2.0;

		expectStateNear(
			state,
			State{
				1.0,
				2.0,
				3.0
			});
	}

	TEST(StateDivisionTests, DivisionAssignmentReturnsReference)
	{
		State state{
			2.0,
			4.0
		};

		State& returned =
			(state /= 2.0);

		EXPECT_EQ(
			&returned,
			&state);
	}

	TEST(StateDivisionTests, DivisionByNegativeScalarWorks)
	{
		const State state{
			2.0,
			-4.0
		};

		expectStateNear(
			state / -2.0,
			State{
				-1.0,
				2.0
			});
	}

	TEST(StateDivisionTests, DivisionRejectsZero)
	{
		const State state{
			1.0,
			2.0
		};

		EXPECT_THROW(
			state / 0.0,
			std::invalid_argument);
	}

	TEST(StateDivisionTests, DivisionAssignmentRejectsZero)
	{
		State state{
			1.0,
			2.0
		};

		EXPECT_THROW(
			state /= 0.0,
			std::invalid_argument);
	}

	// ============================================================
	// isFinite
	// ============================================================

	TEST(StateFiniteTests, FiniteStateIsFinite)
	{
		const State state{
			1.0,
			-2.0,
			3.5,
			0.0
		};

		EXPECT_TRUE(
			state.isFinite());
	}

	TEST(StateFiniteTests, EmptyStateIsFinite)
	{
		const State state;

		EXPECT_TRUE(
			state.isFinite());
	}

	TEST(StateFiniteTests, NaNStateIsNotFinite)
	{
		const State state{
			1.0,
			std::numeric_limits<double>::quiet_NaN(),
			3.0
		};

		EXPECT_FALSE(
			state.isFinite());
	}

	TEST(StateFiniteTests, PositiveInfinityStateIsNotFinite)
	{
		const State state{
			1.0,
			std::numeric_limits<double>::infinity()
		};

		EXPECT_FALSE(
			state.isFinite());
	}

	TEST(StateFiniteTests, NegativeInfinityStateIsNotFinite)
	{
		const State state{
			1.0,
			-std::numeric_limits<double>::infinity()
		};

		EXPECT_FALSE(
			state.isFinite());
	}

	// ============================================================
	// isApprox
	// ============================================================

	TEST(StateApproxTests, IdenticalStatesAreApproximate)
	{
		const State a{
			1.0,
			2.0,
			3.0
		};

		EXPECT_TRUE(
			a.isApprox(
				a,
				EPSILON));
	}

	TEST(StateApproxTests, SmallDifferenceWithinToleranceIsApproximate)
	{
		const State a{
			1.0,
			2.0,
			3.0
		};

		const State b{
			1.0 + 1e-12,
			2.0,
			3.0
		};

		EXPECT_TRUE(
			a.isApprox(
				b,
				1e-9));
	}

	TEST(StateApproxTests, DifferenceOutsideToleranceIsNotApproximate)
	{
		const State a{
			1.0,
			2.0,
			3.0
		};

		const State b{
			1.1,
			2.0,
			3.0
		};

		EXPECT_FALSE(
			a.isApprox(
				b,
				1e-6));
	}

	TEST(StateApproxTests, DifferentSizedStatesAreNotApproximate)
	{
		const State a{
			1.0,
			2.0
		};

		const State b{
			1.0,
			2.0,
			3.0
		};

		EXPECT_FALSE(
			a.isApprox(
				b,
				EPSILON));
	}

	TEST(StateApproxTests, EmptyStatesAreApproximate)
	{
		const State a;
		const State b;

		EXPECT_TRUE(
			a.isApprox(
				b,
				EPSILON));
	}

	TEST(StateApproxTests, RelativeToleranceWorksForLargeValues)
	{
		const State a{
			1e12,
			-2e12
		};

		const State b{
			1e12 + 100.0,
			-2e12 - 100.0
		};

		EXPECT_TRUE(
			a.isApprox(
				b,
				1e-9));
	}

	TEST(StateApproxTests, NonFiniteStatesAreNotApproximate)
	{
		const State a{
			1.0,
			2.0
		};

		const State b{
			1.0,
			std::numeric_limits<double>::infinity()
		};

		EXPECT_FALSE(
			a.isApprox(
				b,
				EPSILON));
	}

	TEST(StateApproxTests, NaNStatesAreNotApproximate)
	{
		const State a{
			1.0,
			2.0
		};

		const State b{
			1.0,
			std::numeric_limits<double>::quiet_NaN()
		};

		EXPECT_FALSE(
			a.isApprox(
				b,
				EPSILON));
	}

	TEST(StateApproxTests, ZeroEpsilonThrows)
	{
		const State state{
			1.0
		};

		EXPECT_THROW(
			state.isApprox(
				state,
				0.0),
			std::invalid_argument);
	}

	TEST(StateApproxTests, NegativeEpsilonThrows)
	{
		const State state{
			1.0
		};

		EXPECT_THROW(
			state.isApprox(
				state,
				-1e-6),
			std::invalid_argument);
	}

	TEST(StateApproxTests, IsApproxIsSymmetric)
	{
		const State a{
			1.0,
			2.0,
			3.0
		};

		const State b{
			1.0 + 1e-11,
			2.0 - 1e-11,
			3.0
		};

		EXPECT_EQ(
			a.isApprox(
				b,
				1e-9),
			b.isApprox(
				a,
				1e-9));
	}

	// ============================================================
	// Algebraic properties
	// ============================================================

	TEST(StateAlgebraTests, AdditionIsCommutative)
	{
		const State a{
			1.0,
			2.0,
			3.0
		};

		const State b{
			-4.0,
			5.0,
			7.0
		};

		expectStateNear(
			a + b,
			b + a);
	}

	TEST(StateAlgebraTests, AdditionIsAssociativeWithinTolerance)
	{
		const State a{
			1.1,
			2.2,
			3.3
		};

		const State b{
			4.4,
			5.5,
			6.6
		};

		const State c{
			7.7,
			8.8,
			9.9
		};

		expectStateNear(
			(a + b) + c,
			a + (b + c));
	}

	TEST(StateAlgebraTests, ScalarDistributivityOverAddition)
	{
		const State a{
			1.0,
			2.0,
			3.0
		};

		const State b{
			4.0,
			-5.0,
			6.0
		};

		const double scalar =
			3.5;

		expectStateNear(
			(a + b) * scalar,
			a * scalar +
			b * scalar);
	}

	TEST(StateAlgebraTests, DivisionReversesMultiplication)
	{
		const State state{
			1.0,
			-2.0,
			7.0
		};

		const double scalar =
			4.5;

		expectStateNear(
			(state * scalar) /
			scalar,
			state);
	}

	// ============================================================
	// Deterministic randomized property tests
	// ============================================================

	TEST(StatePropertyTests, RandomizedArithmeticProperties)
	{
		std::mt19937_64 generator(
			0xA3E0F04EULL);

		std::uniform_real_distribution<double>
			valueDistribution(
				-1000.0,
				1000.0);

		std::uniform_real_distribution<double>
			scalarDistribution(
				0.1,
				10.0);

		constexpr std::size_t STATE_SIZE =
			16;

		for (int iteration = 0;
			iteration < 1000;
			++iteration)
		{
			State a(STATE_SIZE);
			State b(STATE_SIZE);

			for (std::size_t i = 0;
				i < STATE_SIZE;
				++i)
			{
				a[i] =
					valueDistribution(generator);

				b[i] =
					valueDistribution(generator);
			}

			const double scalar =
				scalarDistribution(generator);

			// a + b = b + a
			EXPECT_TRUE(
				(a + b).isApprox(
					b + a,
					1e-10));

			// a - a = 0
			const State zero(STATE_SIZE);

			EXPECT_TRUE(
				(a - a).isApprox(
					zero,
					1e-10));

			// a * 1 = a
			EXPECT_TRUE(
				(a * 1.0).isApprox(
					a,
					1e-10));

			// a * 0 = 0
			EXPECT_TRUE(
				(a * 0.0).isApprox(
					zero,
					1e-10));

			// scalar * a = a * scalar
			EXPECT_TRUE(
				(scalar * a).isApprox(
					a * scalar,
					1e-10));

			// scalar distributivity
			EXPECT_TRUE(
				((a + b) * scalar)
				.isApprox(
					a * scalar +
					b * scalar,
					1e-9));

			// multiply then divide restores state
			EXPECT_TRUE(
				((a * scalar) / scalar)
				.isApprox(
					a,
					1e-9));

			// generated states remain finite
			EXPECT_TRUE(
				a.isFinite());

			EXPECT_TRUE(
				b.isFinite());
		}
	}
}