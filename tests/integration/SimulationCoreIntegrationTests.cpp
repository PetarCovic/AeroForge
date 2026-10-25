#include <gtest/gtest.h>

#include <simulation/DynamicsModel.h>
#include <simulation/EulerIntegrator.h>
#include <simulation/Integrator.h>
#include <simulation/RK4Integrator.h>
#include <simulation/SimulationRunner.h>

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace aeroforge::simulation::tests
{
	namespace
	{
		class ConstantAccelerationModel final :
			public DynamicsModel
		{
		private:
			double acceleration;

		public:
			explicit ConstantAccelerationModel(
				double acceleration)
				: acceleration(acceleration)
			{
			}

			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				(void)time;

				if (state.size() != 2)
				{
					throw std::invalid_argument(
						"ConstantAccelerationModel requires state size 2.");
				}

				// state[0] = position
				// state[1] = velocity
				//
				// dx/dt = velocity
				// dv/dt = acceleration

				return StateDerivative{
					state[1],
					acceleration
				};
			}
		};

		class ExponentialModel final :
			public DynamicsModel
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

		void expectStateBitwiseEqual(
			const State& a,
			const State& b)
		{
			ASSERT_EQ(
				a.size(),
				b.size());

			for (std::size_t i = 0;
				i < a.size();
				++i)
			{
				EXPECT_EQ(
					std::bit_cast<std::uint64_t>(
						a[i]),
					std::bit_cast<std::uint64_t>(
						b[i]));
			}
		}

		State runUsingIntegratorInterface(
			const Integrator& integrator,
			const DynamicsModel& model,
			const State& initialState,
			double dt,
			std::uint64_t steps)
		{
			SimulationRunner runner(
				model,
				integrator,
				initialState,
				dt);

			runner.runSteps(
				steps);

			return runner.getState();
		}
	}

	TEST(
		SimulationCoreIntegrationTests,
		RK4MatchesAnalyticalConstantAccelerationSolution)
	{
		constexpr double ACCELERATION =
			-9.81;

		constexpr double INITIAL_POSITION =
			100.0;

		constexpr double INITIAL_VELOCITY =
			20.0;

		constexpr double DT =
			0.01;

		constexpr std::uint64_t STEPS =
			1000;

		ConstantAccelerationModel model(
			ACCELERATION);

		RK4Integrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				INITIAL_POSITION,
				INITIAL_VELOCITY
			},
			DT);

		runner.runSteps(STEPS);

		const double time =
			runner.getCurrentTime();

		const double expectedPosition =
			INITIAL_POSITION
			+ INITIAL_VELOCITY * time
			+ 0.5
			* ACCELERATION
			* time
			* time;

		const double expectedVelocity =
			INITIAL_VELOCITY
			+ ACCELERATION
			* time;

		EXPECT_NEAR(
			runner.getState()[0],
			expectedPosition,
			1e-8);

		EXPECT_NEAR(
			runner.getState()[1],
			expectedVelocity,
			1e-8);
	}

	TEST(
		SimulationCoreIntegrationTests,
		StateClockAndStepCountRemainSynchronized)
	{
		ConstantAccelerationModel model(
			1.5);

		RK4Integrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0,
				0.0
			},
			0.005);

		runner.runSteps(4000);

		EXPECT_EQ(
			runner.getCurrentStep(),
			4000U);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			20.0);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			static_cast<double>(
				runner.getCurrentStep())
			* runner.getClock()
			.getTimeStep());

		EXPECT_TRUE(
			runner.getState()
			.isFinite());
	}

	TEST(
		SimulationCoreIntegrationTests,
		IntegratorImplementationsAreInterchangeable)
	{
		ExponentialModel model;

		EulerIntegrator euler;
		RK4Integrator rk4;

		const State initialState{
			1.0
		};

		const State eulerResult =
			runUsingIntegratorInterface(
				euler,
				model,
				initialState,
				0.01,
				100);

		const State rk4Result =
			runUsingIntegratorInterface(
				rk4,
				model,
				initialState,
				0.01,
				100);

		EXPECT_TRUE(
			eulerResult.isFinite());

		EXPECT_TRUE(
			rk4Result.isFinite());

		const double exact =
			std::exp(1.0);

		const double eulerError =
			std::abs(
				eulerResult[0]
				- exact);

		const double rk4Error =
			std::abs(
				rk4Result[0]
				- exact);

		EXPECT_LT(
			rk4Error,
			eulerError);
	}

	TEST(
		SimulationCoreIntegrationTests,
		ResetAndReplayProducesIdenticalTrajectoryEndpoint)
	{
		ConstantAccelerationModel model(
			-3.5);

		RK4Integrator integrator;

		const State initialState{
			250.0,
			75.0
		};

		SimulationRunner runner(
			model,
			integrator,
			initialState,
			0.002);

		runner.runSteps(5000);

		const State firstResult =
			runner.getState();

		const std::uint64_t firstStep =
			runner.getCurrentStep();

		const double firstTime =
			runner.getCurrentTime();

		runner.reset(initialState);

		runner.runSteps(5000);

		expectStateBitwiseEqual(
			firstResult,
			runner.getState());

		EXPECT_EQ(
			runner.getCurrentStep(),
			firstStep);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			firstTime);
	}

	TEST(
		SimulationCoreIntegrationTests,
		LongRK4SimulationRemainsFinite)
	{
		ConstantAccelerationModel model(
			0.01);

		RK4Integrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0,
				1.0
			},
			0.001);

		runner.runSteps(100000);

		EXPECT_TRUE(
			runner.getState()
			.isFinite());

		EXPECT_EQ(
			runner.getCurrentStep(),
			100000U);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			100.0);
	}

	TEST(
		SimulationCoreIntegrationTests,
		DifferentRK4TimeStepsConvergeToSamePhysicalSolution)
	{
		ExponentialModel model;
		RK4Integrator integrator;

		SimulationRunner coarse(
			model,
			integrator,
			State{
				1.0
			},
			0.01);

		SimulationRunner fine(
			model,
			integrator,
			State{
				1.0
			},
			0.005);

		coarse.runSteps(100);
		fine.runSteps(200);

		EXPECT_DOUBLE_EQ(
			coarse.getCurrentTime(),
			1.0);

		EXPECT_DOUBLE_EQ(
			fine.getCurrentTime(),
			1.0);

		const double exact =
			std::exp(1.0);

		EXPECT_NEAR(
			coarse.getState()[0],
			exact,
			1e-8);

		EXPECT_NEAR(
			fine.getState()[0],
			exact,
			1e-9);
	}

	TEST(
		SimulationCoreIntegrationTests,
		CompleteSimulationIsRepeatableAcrossIndependentRunners)
	{
		ConstantAccelerationModel model(
			-9.81);

		RK4Integrator integrator;

		const State initialState{
			1000.0,
			150.0
		};

		SimulationRunner runnerA(
			model,
			integrator,
			initialState,
			0.001);

		SimulationRunner runnerB(
			model,
			integrator,
			initialState,
			0.001);

		runnerA.runSteps(20000);
		runnerB.runSteps(20000);

		expectStateBitwiseEqual(
			runnerA.getState(),
			runnerB.getState());

		EXPECT_EQ(
			runnerA.getCurrentStep(),
			runnerB.getCurrentStep());

		EXPECT_DOUBLE_EQ(
			runnerA.getCurrentTime(),
			runnerB.getCurrentTime());
	}
}