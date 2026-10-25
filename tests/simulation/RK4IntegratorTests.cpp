#include <gtest/gtest.h>

#include <simulation/DynamicsModel.h>
#include <simulation/EulerIntegrator.h>
#include <simulation/RK4Integrator.h>

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

		class TimeDependentModel final : public DynamicsModel
		{
		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				StateDerivative derivative(state.size());

				for (std::size_t i = 0;
					i < derivative.size();
					++i)
				{
					derivative[i] = time;
				}

				return derivative;
			}
		};

		class HarmonicOscillatorModel final : public DynamicsModel
		{
		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				(void)time;

				if (state.size() != 2)
				{
					throw std::invalid_argument(
						"Harmonic oscillator requires state size 2.");
				}

				// state[0] = position
				// state[1] = velocity
				//
				// dx/dt = v
				// dv/dt = -x

				return StateDerivative{
					state[1],
					-state[0]
				};
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

				StateDerivative derivative(state.size());

				if (!derivative.empty())
				{
					derivative[0] =
						std::numeric_limits<double>::quiet_NaN();
				}

				return derivative;
			}
		};

		class FailsOnSecondEvaluationModel final : public DynamicsModel
		{
		private:
			mutable int evaluationCount = 0;

		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				(void)time;

				++evaluationCount;

				if (evaluationCount == 2)
				{
					StateDerivative derivative(state.size());

					if (!derivative.empty())
					{
						derivative[0] =
							std::numeric_limits<double>::
							quiet_NaN();
					}

					return derivative;
				}

				return StateDerivative(state.size());
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
	// Basic RK4 behavior
	// ============================================================

	TEST(
		RK4IntegratorTests,
		IntegratesConstantDerivativeExactly)
	{
		RK4Integrator integrator;

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
		RK4IntegratorTests,
		UsesStateDependentDynamics)
	{
		RK4Integrator integrator;
		StateDependentModel model;

		const State state{
			1.0
		};

		const double dt = 0.1;

		const State result =
			integrator.step(
				model,
				0.0,
				state,
				dt);

		const double exact =
			std::exp(dt);

		EXPECT_NEAR(
			result[0],
			exact,
			1e-6);
	}

	TEST(
		RK4IntegratorTests,
		UsesTimeDependentDynamics)
	{
		RK4Integrator integrator;
		TimeDependentModel model;

		const State state{
			0.0
		};

		const State result =
			integrator.step(
				model,
				2.0,
				state,
				0.5);

		// dx/dt = t
		//
		// Integral from 2.0 to 2.5:
		// 0.5 * (2.5^2 - 2.0^2)
		const double expected =
			0.5 *
			(
				2.5 * 2.5 -
				2.0 * 2.0
				);

		EXPECT_NEAR(
			result[0],
			expected,
			1e-12);
	}

	TEST(
		RK4IntegratorTests,
		DoesNotModifyInputState)
	{
		RK4Integrator integrator;

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
				0.1);

		(void)result;

		expectStateNear(
			state,
			State{
				5.0,
				10.0
			});
	}

	// ============================================================
	// Input validation
	// ============================================================

	TEST(
		RK4IntegratorTests,
		RejectsZeroTimeStep)
	{
		RK4Integrator integrator;

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
		RK4IntegratorTests,
		RejectsNegativeTimeStep)
	{
		RK4Integrator integrator;

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
		RK4IntegratorTests,
		RejectsNaNTimeStep)
	{
		RK4Integrator integrator;

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
		RK4IntegratorTests,
		RejectsInfiniteTimeStep)
	{
		RK4Integrator integrator;

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

	TEST(
		RK4IntegratorTests,
		RejectsNaNTime)
	{
		RK4Integrator integrator;

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
		RK4IntegratorTests,
		RejectsInfiniteTime)
	{
		RK4Integrator integrator;

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

	TEST(
		RK4IntegratorTests,
		RejectsNonFiniteInputState)
	{
		RK4Integrator integrator;

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

	// ============================================================
	// Derivative validation
	// ============================================================

	TEST(
		RK4IntegratorTests,
		RejectsWrongDerivativeDimension)
	{
		RK4Integrator integrator;
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
		RK4IntegratorTests,
		RejectsNonFiniteDerivative)
	{
		RK4Integrator integrator;
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

	TEST(
		RK4IntegratorTests,
		RejectsInvalidIntermediateDerivative)
	{
		RK4Integrator integrator;
		FailsOnSecondEvaluationModel model;

		const State state{
			1.0
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
		RK4IntegratorTests,
		AllowsEmptyStateWithEmptyDerivative)
	{
		RK4Integrator integrator;

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
		RK4IntegratorTests,
		ZeroDerivativeLeavesStateUnchanged)
	{
		RK4Integrator integrator;

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
				50.0,
				state,
				0.25);

		expectStateNear(
			result,
			state);
	}

	// ============================================================
	// Analytical validation
	// ============================================================

	TEST(
		RK4IntegratorTests,
		AccuratelyIntegratesExponentialGrowth)
	{
		RK4Integrator integrator;
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

		const double exact =
			std::exp(1.0);

		EXPECT_NEAR(
			state[0],
			exact,
			1e-5);
	}

	TEST(
		RK4IntegratorTests,
		AccuratelyIntegratesHarmonicOscillator)
	{
		RK4Integrator integrator;
		HarmonicOscillatorModel model;

		State state{
			1.0,
			0.0
		};

		double time = 0.0;

		constexpr double DT =
			0.01;

		const double finalTime =
			2.0 * std::acos(-1.0);

		const int steps =
			static_cast<int>(
				finalTime / DT);

		for (int i = 0;
			i < steps;
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

		const double expectedPosition =
			std::cos(time);

		const double expectedVelocity =
			-std::sin(time);

		EXPECT_NEAR(
			state[0],
			expectedPosition,
			1e-7);

		EXPECT_NEAR(
			state[1],
			expectedVelocity,
			1e-7);
	}

	// ============================================================
	// Euler vs RK4
	// ============================================================

	TEST(
		RK4IntegratorTests,
		RK4IsMoreAccurateThanEulerForExponentialGrowth)
	{
		RK4Integrator rk4;
		EulerIntegrator euler;

		StateDependentModel model;

		State rk4State{
			1.0
		};

		State eulerState{
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
			rk4State =
				rk4.step(
					model,
					time,
					rk4State,
					DT);

			eulerState =
				euler.step(
					model,
					time,
					eulerState,
					DT);

			time += DT;
		}

		const double exact =
			std::exp(1.0);

		const double rk4Error =
			std::abs(
				rk4State[0] - exact);

		const double eulerError =
			std::abs(
				eulerState[0] - exact);

		EXPECT_LT(
			rk4Error,
			eulerError);
	}

	TEST(
		RK4IntegratorTests,
		RK4IsSubstantiallyMoreAccurateThanEuler)
	{
		RK4Integrator rk4;
		EulerIntegrator euler;

		StateDependentModel model;

		State rk4State{
			1.0
		};

		State eulerState{
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
			rk4State =
				rk4.step(
					model,
					time,
					rk4State,
					DT);

			eulerState =
				euler.step(
					model,
					time,
					eulerState,
					DT);

			time += DT;
		}

		const double exact =
			std::exp(1.0);

		const double rk4Error =
			std::abs(
				rk4State[0] - exact);

		const double eulerError =
			std::abs(
				eulerState[0] - exact);

		EXPECT_LT(
			rk4Error,
			eulerError * 0.01);
	}

	// ============================================================
	// RK4 convergence
	// ============================================================

	TEST(
		RK4IntegratorTests,
		SmallerTimeStepImprovesAccuracy)
	{
		RK4Integrator integrator;
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
				0.2,
				5);

		const double fine =
			integrate(
				0.1,
				10);

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

	TEST(
		RK4IntegratorTests,
		ExhibitsApproximatelyFourthOrderConvergence)
	{
		RK4Integrator integrator;
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
				0.2,
				5);

		const double fine =
			integrate(
				0.1,
				10);

		const double exact =
			std::exp(1.0);

		const double coarseError =
			std::abs(
				coarse - exact);

		const double fineError =
			std::abs(
				fine - exact);

		const double errorRatio =
			coarseError / fineError;

		// Halving dt for a fourth-order method should
		// reduce global error by approximately 2^4 = 16.
		EXPECT_GT(
			errorRatio,
			12.0);

		EXPECT_LT(
			errorRatio,
			20.0);
	}
}