#include <gtest/gtest.h>

#include <simulation/DynamicsModel.h>
#include <simulation/EulerIntegrator.h>
#include <simulation/RK4Integrator.h>
#include <simulation/SimulationRunner.h>

#include <cmath>
#include <vector>
#include <limits>

namespace aeroforge::simulation::tests
{
	namespace
	{
		class ExponentialGrowthModel final : public DynamicsModel
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

		class HarmonicOscillatorModel final : public DynamicsModel
		{
		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				(void)time;

				return StateDerivative{
					state[1],
					-state[0]
				};
			}
		};

		double integrateExponential(
			const Integrator& integrator,
			double dt)
		{
			ExponentialGrowthModel model;

			SimulationRunner runner(
				model,
				integrator,
				State{
					1.0
				},
				dt);

			const std::uint64_t steps =
				static_cast<std::uint64_t>(
					std::llround(1.0 / dt));

			runner.runSteps(steps);

			return runner.getState()[0];
		}

		State integrateOscillator(
			const Integrator& integrator,
			double dt)
		{
			HarmonicOscillatorModel model;

			SimulationRunner runner(
				model,
				integrator,
				State{
					1.0,
					0.0
				},
				dt);

			const std::uint64_t steps =
				static_cast<std::uint64_t>(
					std::llround(1.0 / dt));

			runner.runSteps(steps);

			return runner.getState();
		}

		double oscillatorError(
			const State& state)
		{
			const double exactPosition =
				std::cos(1.0);

			const double exactVelocity =
				-std::sin(1.0);

			const double positionError =
				state[0] - exactPosition;

			const double velocityError =
				state[1] - exactVelocity;

			return std::sqrt(
				positionError * positionError +
				velocityError * velocityError);
		}
	}

	// ============================================================
	// Euler convergence
	// ============================================================

	TEST(
		NumericalAccuracyTests,
		EulerErrorDecreasesAsTimeStepShrinks)
	{
		EulerIntegrator integrator;

		const std::vector<double> timeSteps{
			0.2,
			0.1,
			0.05,
			0.025
		};

		const double exact =
			std::exp(1.0);

		double previousError =
			std::numeric_limits<double>::infinity();

		for (double dt : timeSteps)
		{
			const double result =
				integrateExponential(
					integrator,
					dt);

			const double error =
				std::abs(
					result - exact);

			EXPECT_LT(
				error,
				previousError);

			previousError =
				error;
		}
	}

	TEST(
		NumericalAccuracyTests,
		EulerShowsApproximatelyFirstOrderConvergence)
	{
		EulerIntegrator integrator;

		const std::vector<double> timeSteps{
			0.2,
			0.1,
			0.05,
			0.025
		};

		const double exact =
			std::exp(1.0);

		std::vector<double> errors;

		for (double dt : timeSteps)
		{
			const double result =
				integrateExponential(
					integrator,
					dt);

			errors.push_back(
				std::abs(
					result - exact));
		}

		for (std::size_t i = 0;
			i + 1 < errors.size();
			++i)
		{
			const double ratio =
				errors[i] /
				errors[i + 1];

			// First-order global convergence:
			// halving dt should reduce error by ~2.
			EXPECT_GT(
				ratio,
				1.7);

			EXPECT_LT(
				ratio,
				2.2);
		}
	}

	// ============================================================
	// RK4 convergence
	// ============================================================

	TEST(
		NumericalAccuracyTests,
		RK4ErrorDecreasesAsTimeStepShrinks)
	{
		RK4Integrator integrator;

		const std::vector<double> timeSteps{
			0.2,
			0.1,
			0.05,
			0.025
		};

		const double exact =
			std::exp(1.0);

		double previousError =
			std::numeric_limits<double>::infinity();

		for (double dt : timeSteps)
		{
			const double result =
				integrateExponential(
					integrator,
					dt);

			const double error =
				std::abs(
					result - exact);

			EXPECT_LT(
				error,
				previousError);

			previousError =
				error;
		}
	}

	TEST(
		NumericalAccuracyTests,
		RK4ShowsApproximatelyFourthOrderConvergence)
	{
		RK4Integrator integrator;

		const std::vector<double> timeSteps{
			0.2,
			0.1,
			0.05,
			0.025
		};

		const double exact =
			std::exp(1.0);

		std::vector<double> errors;

		for (double dt : timeSteps)
		{
			const double result =
				integrateExponential(
					integrator,
					dt);

			errors.push_back(
				std::abs(
					result - exact));
		}

		for (std::size_t i = 0;
			i + 1 < errors.size();
			++i)
		{
			const double ratio =
				errors[i] /
				errors[i + 1];

			// Fourth-order global convergence:
			// halving dt should reduce error by ~16.
			EXPECT_GT(
				ratio,
				12.0);

			EXPECT_LT(
				ratio,
				20.0);
		}
	}

	// ============================================================
	// Euler vs RK4
	// ============================================================

	TEST(
		NumericalAccuracyTests,
		RK4IsMoreAccurateThanEulerAtSameTimeStep)
	{
		EulerIntegrator euler;
		RK4Integrator rk4;

		constexpr double DT =
			0.1;

		const double exact =
			std::exp(1.0);

		const double eulerResult =
			integrateExponential(
				euler,
				DT);

		const double rk4Result =
			integrateExponential(
				rk4,
				DT);

		const double eulerError =
			std::abs(
				eulerResult - exact);

		const double rk4Error =
			std::abs(
				rk4Result - exact);

		EXPECT_LT(
			rk4Error,
			eulerError);
	}

	TEST(
		NumericalAccuracyTests,
		RK4IsSubstantiallyMoreAccurateThanEuler)
	{
		EulerIntegrator euler;
		RK4Integrator rk4;

		constexpr double DT =
			0.1;

		const double exact =
			std::exp(1.0);

		const double eulerError =
			std::abs(
				integrateExponential(
					euler,
					DT)
				- exact);

		const double rk4Error =
			std::abs(
				integrateExponential(
					rk4,
					DT)
				- exact);

		EXPECT_LT(
			rk4Error,
			eulerError * 0.01);
	}

	// ============================================================
	// Harmonic oscillator
	// ============================================================

	TEST(
		NumericalAccuracyTests,
		RK4AccuratelyIntegratesHarmonicOscillator)
	{
		RK4Integrator integrator;

		const State result =
			integrateOscillator(
				integrator,
				0.01);

		EXPECT_NEAR(
			result[0],
			std::cos(1.0),
			1e-8);

		EXPECT_NEAR(
			result[1],
			-std::sin(1.0),
			1e-8);
	}

	TEST(
		NumericalAccuracyTests,
		RK4OscillatorErrorDecreasesWithSmallerTimeStep)
	{
		RK4Integrator integrator;

		const State coarse =
			integrateOscillator(
				integrator,
				0.2);

		const State fine =
			integrateOscillator(
				integrator,
				0.1);

		EXPECT_LT(
			oscillatorError(fine),
			oscillatorError(coarse));
	}

	TEST(
		NumericalAccuracyTests,
		RK4ShowsFourthOrderConvergenceForOscillator)
	{
		RK4Integrator integrator;

		const std::vector<double> timeSteps{
			0.2,
			0.1,
			0.05,
			0.025
		};

		std::vector<double> errors;

		for (double dt : timeSteps)
		{
			errors.push_back(
				oscillatorError(
					integrateOscillator(
						integrator,
						dt)));
		}

		for (std::size_t i = 0;
			i + 1 < errors.size();
			++i)
		{
			const double ratio =
				errors[i] /
				errors[i + 1];

			EXPECT_GT(
				ratio,
				12.0);

			EXPECT_LT(
				ratio,
				20.0);
		}
	}

	TEST(
		NumericalAccuracyTests,
		RK4OutperformsEulerForOscillatorySystem)
	{
		EulerIntegrator euler;
		RK4Integrator rk4;

		constexpr double DT =
			0.1;

		const double eulerError =
			oscillatorError(
				integrateOscillator(
					euler,
					DT));

		const double rk4Error =
			oscillatorError(
				integrateOscillator(
					rk4,
					DT));

		EXPECT_LT(
			rk4Error,
			eulerError);
	}

	// ============================================================
	// Repeatability
	// ============================================================

	TEST(
		NumericalAccuracyTests,
		RepeatedRK4RunsProduceIdenticalResults)
	{
		RK4Integrator integrator;

		const State first =
			integrateOscillator(
				integrator,
				0.01);

		const State second =
			integrateOscillator(
				integrator,
				0.01);

		EXPECT_DOUBLE_EQ(
			first[0],
			second[0]);

		EXPECT_DOUBLE_EQ(
			first[1],
			second[1]);
	}
}