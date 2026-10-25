#include <gtest/gtest.h>

#include <simulation/DynamicsModel.h>
#include <simulation/EulerIntegrator.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace aeroforge::simulation::tests
{
	namespace
	{
		constexpr double EPSILON = 1e-10;

		class ConstantDerivativeModel final : public DynamicsModel
		{
		private:
			StateDerivative derivative;

		public:
			explicit ConstantDerivativeModel(
				const StateDerivative& derivative)
				: derivative(derivative)
			{
			}

			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				(void)time;
				(void)state;

				return derivative;
			}
		};

		class TimeDependentModel final : public DynamicsModel
		{
		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				StateDerivative derivative(
					state.size());

				for (std::size_t i = 0;
					i < derivative.size();
					++i)
				{
					derivative[i] = time;
				}

				return derivative;
			}
		};

		class StateDependentModel final : public DynamicsModel
		{
		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				(void)time;

				return state;
			}
		};

		class WrongSizeModel final : public DynamicsModel
		{
		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				(void)time;

				return StateDerivative(
					state.size() + 1);
			}
		};

		class NonFiniteDerivativeModel final : public DynamicsModel
		{
		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				(void)time;

				StateDerivative derivative(
					state.size());

				if (!derivative.empty())
				{
					derivative[0] =
						std::numeric_limits<double>::
						quiet_NaN();
				}

				return derivative;
			}
		};

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
	// Basic Euler integration
	// ============================================================

	TEST(
		EulerIntegratorTests,
		IntegratesConstantDerivative)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				2.0,
				-4.0,
				1.0
			});

		const State state{
			10.0,
			5.0,
			-3.0
		};

		const State result =
			integrator.step(
				model,
				0.0,
				state,
				0.5);

		expectStateNear(
			result,
			State{
				11.0,
				3.0,
				-2.5
			});
	}

	TEST(
		EulerIntegratorTests,
		UsesCurrentStateWhenEvaluatingDerivative)
	{
		EulerIntegrator integrator;
		StateDependentModel model;

		const State state{
			2.0,
			-3.0
		};

		const State result =
			integrator.step(
				model,
				0.0,
				state,
				0.25);

		expectStateNear(
			result,
			State{
				2.5,
				-3.75
			});
	}

	TEST(
		EulerIntegratorTests,
		UsesCurrentTimeWhenEvaluatingDerivative)
	{
		EulerIntegrator integrator;
		TimeDependentModel model;

		const State state{
			1.0,
			2.0
		};

		const State result =
			integrator.step(
				model,
				4.0,
				state,
				0.5);

		expectStateNear(
			result,
			State{
				3.0,
				4.0
			});
	}

	TEST(
		EulerIntegratorTests,
		DoesNotModifyInputState)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0,
				2.0
			});

		const State state{
			5.0,
			10.0
		};

		const State result =
			integrator.step(
				model,
				0.0,
				state,
				1.0);

		(void)result;

		expectStateNear(
			state,
			State{
				5.0,
				10.0
			});
	}

	// ============================================================
	// Time-step validation
	// ============================================================

	TEST(
		EulerIntegratorTests,
		RejectsZeroTimeStep)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		const State state{
			0.0
		};

		EXPECT_THROW(
			integrator.step(
				model,
				0.0,
				state,
				0.0),
			std::invalid_argument);
	}

	TEST(
		EulerIntegratorTests,
		RejectsNegativeTimeStep)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		const State state{
			0.0
		};

		EXPECT_THROW(
			integrator.step(
				model,
				0.0,
				state,
				-0.1),
			std::invalid_argument);
	}

	TEST(
		EulerIntegratorTests,
		RejectsNaNTimeStep)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		const State state{
			0.0
		};

		EXPECT_THROW(
			integrator.step(
				model,
				0.0,
				state,
				std::numeric_limits<double>::
				quiet_NaN()),
			std::invalid_argument);
	}

	TEST(
		EulerIntegratorTests,
		RejectsInfiniteTimeStep)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		const State state{
			0.0
		};

		EXPECT_THROW(
			integrator.step(
				model,
				0.0,
				state,
				std::numeric_limits<double>::
				infinity()),
			std::invalid_argument);
	}

	// ============================================================
	// Time validation
	// ============================================================

	TEST(
		EulerIntegratorTests,
		RejectsNaNTime)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		const State state{
			0.0
		};

		EXPECT_THROW(
			integrator.step(
				model,
				std::numeric_limits<double>::
				quiet_NaN(),
				state,
				0.1),
			std::invalid_argument);
	}

	TEST(
		EulerIntegratorTests,
		RejectsInfiniteTime)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		const State state{
			0.0
		};

		EXPECT_THROW(
			integrator.step(
				model,
				std::numeric_limits<double>::
				infinity(),
				state,
				0.1),
			std::invalid_argument);
	}

	// ============================================================
	// State validation
	// ============================================================

	TEST(
		EulerIntegratorTests,
		RejectsNaNState)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0,
				1.0
			});

		const State state{
			1.0,
			std::numeric_limits<double>::
				quiet_NaN()
		};

		EXPECT_THROW(
			integrator.step(
				model,
				0.0,
				state,
				0.1),
			std::invalid_argument);
	}

	TEST(
		EulerIntegratorTests,
		RejectsInfiniteState)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				1.0,
				1.0
			});

		const State state{
			1.0,
			std::numeric_limits<double>::
				infinity()
		};

		EXPECT_THROW(
			integrator.step(
				model,
				0.0,
				state,
				0.1),
			std::invalid_argument);
	}

	// ============================================================
	// Dynamics-model validation
	// ============================================================

	TEST(
		EulerIntegratorTests,
		RejectsWrongDerivativeDimension)
	{
		EulerIntegrator integrator;
		WrongSizeModel model;

		const State state{
			1.0,
			2.0
		};

		EXPECT_THROW(
			integrator.step(
				model,
				0.0,
				state,
				0.1),
			std::invalid_argument);
	}

	TEST(
		EulerIntegratorTests,
		RejectsNonFiniteDerivative)
	{
		EulerIntegrator integrator;
		NonFiniteDerivativeModel model;

		const State state{
			1.0,
			2.0
		};

		EXPECT_THROW(
			integrator.step(
				model,
				0.0,
				state,
				0.1),
			std::invalid_argument);
	}

	// ============================================================
	// Edge cases
	// ============================================================

	TEST(
		EulerIntegratorTests,
		AllowsEmptyStateWithEmptyDerivative)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{});

		const State state;

		const State result =
			integrator.step(
				model,
				0.0,
				state,
				0.1);

		EXPECT_TRUE(
			result.empty());
	}

	TEST(
		EulerIntegratorTests,
		ZeroDerivativeLeavesStateUnchanged)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				0.0,
				0.0,
				0.0
			});

		const State state{
			1.0,
			-4.0,
			8.0
		};

		const State result =
			integrator.step(
				model,
				100.0,
				state,
				0.25);

		expectStateNear(
			result,
			state);
	}

	// ============================================================
	// Numerical behavior
	// ============================================================

	TEST(
		EulerIntegratorTests,
		RepeatedConstantVelocityIntegrationIsExact)
	{
		EulerIntegrator integrator;

		ConstantDerivativeModel model(
			StateDerivative{
				5.0
			});

		State state{
			0.0
		};

		double time = 0.0;

		constexpr double DT =
			0.01;

		constexpr int STEPS =
			1000;

		for (int i = 0;
			i < STEPS;
			++i)
		{
			state =
				integrator.step(
					model,
					time,
					state,
					DT);

			time += DT;
		}

		EXPECT_NEAR(
			state[0],
			50.0,
			1e-9);
	}

	TEST(
		EulerIntegratorTests,
		ExponentialGrowthShowsExpectedEulerResult)
	{
		EulerIntegrator integrator;
		StateDependentModel model;

		State state{
			1.0
		};

		double time = 0.0;

		constexpr double DT =
			0.1;

		constexpr int STEPS =
			10;

		for (int i = 0;
			i < STEPS;
			++i)
		{
			state =
				integrator.step(
					model,
					time,
					state,
					DT);

			time += DT;
		}

		// Euler for dx/dt = x:
		// x_n = (1 + dt)^n x_0
		const double expected =
			std::pow(
				1.0 + DT,
				STEPS);

		EXPECT_NEAR(
			state[0],
			expected,
			1e-12);
	}

	TEST(
		EulerIntegratorTests,
		SmallerTimeStepImprovesExponentialAccuracy)
	{
		EulerIntegrator integrator;
		StateDependentModel model;

		auto integrate =
			[&integrator, &model](
				double dt,
				int steps)
			{
				State state{
					1.0
				};

				double time =
					0.0;

				for (int i = 0;
					i < steps;
					++i)
				{
					state =
						integrator.step(
							model,
							time,
							state,
							dt);

					time += dt;
				}

				return state[0];
			};

		const double coarse =
			integrate(
				0.1,
				10);

		const double fine =
			integrate(
				0.01,
				100);

		const double exact =
			std::exp(1.0);

		const double coarseError =
			std::abs(
				coarse - exact);

		const double fineError =
			std::abs(
				fine - exact);

		EXPECT_LT(
			fineError,
			coarseError);
	}
}