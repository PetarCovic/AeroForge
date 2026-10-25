#include <gtest/gtest.h>

#include <simulation/DynamicsModel.h>
#include <simulation/EulerIntegrator.h>
#include <simulation/RK4Integrator.h>
#include <simulation/SimulationRunner.h>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace aeroforge::simulation::tests
{
	namespace
	{
		class DeterministicModel final : public DynamicsModel
		{
		public:
			StateDerivative evaluate(
				double time,
				const State& state) const override
			{
				if (state.size() != 3)
				{
					throw std::invalid_argument(
						"DeterministicModel requires state size 3.");
				}

				return StateDerivative{
					state[1],
					-2.0 * state[0]
						- 0.15 * state[1],
					-0.05 * state[2]
						+ 0.001 * time
				};
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
				const std::uint64_t bitsA =
					std::bit_cast<std::uint64_t>(
						a[i]);

				const std::uint64_t bitsB =
					std::bit_cast<std::uint64_t>(
						b[i]);

				EXPECT_EQ(
					bitsA,
					bitsB);
			}
		}
	}

	TEST(
		DeterminismTests,
		IdenticalRK4RunsProduceBitwiseIdenticalStates)
	{
		DeterministicModel model;
		RK4Integrator integrator;

		const State initialState{
			10.0,
			-2.0,
			5.0
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

		runnerA.runSteps(10000);
		runnerB.runSteps(10000);

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

	TEST(
		DeterminismTests,
		IdenticalEulerRunsProduceBitwiseIdenticalStates)
	{
		DeterministicModel model;
		EulerIntegrator integrator;

		const State initialState{
			10.0,
			-2.0,
			5.0
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

		runnerA.runSteps(10000);
		runnerB.runSteps(10000);

		expectStateBitwiseEqual(
			runnerA.getState(),
			runnerB.getState());
	}

	TEST(
		DeterminismTests,
		RunStepsMatchesManualSteppingBitForBit)
	{
		DeterministicModel model;
		RK4Integrator integrator;

		const State initialState{
			3.0,
			-1.0,
			8.0
		};

		SimulationRunner batchRunner(
			model,
			integrator,
			initialState,
			0.002);

		SimulationRunner manualRunner(
			model,
			integrator,
			initialState,
			0.002);

		constexpr std::uint64_t STEPS =
			5000;

		batchRunner.runSteps(STEPS);

		for (std::uint64_t i = 0;
			i < STEPS;
			++i)
		{
			manualRunner.step();
		}

		expectStateBitwiseEqual(
			batchRunner.getState(),
			manualRunner.getState());

		EXPECT_EQ(
			batchRunner.getCurrentStep(),
			manualRunner.getCurrentStep());

		EXPECT_DOUBLE_EQ(
			batchRunner.getCurrentTime(),
			manualRunner.getCurrentTime());
	}

	TEST(
		DeterminismTests,
		ResetAndRerunReproducesOriginalResult)
	{
		DeterministicModel model;
		RK4Integrator integrator;

		const State initialState{
			2.0,
			4.0,
			6.0
		};

		SimulationRunner runner(
			model,
			integrator,
			initialState,
			0.001);

		runner.runSteps(10000);

		const State firstResult =
			runner.getState();

		const double firstTime =
			runner.getCurrentTime();

		runner.reset(initialState);

		runner.runSteps(10000);

		expectStateBitwiseEqual(
			firstResult,
			runner.getState());

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			firstTime);
	}

	TEST(
		DeterminismTests,
		SameNonzeroStartStepProducesSameResult)
	{
		DeterministicModel model;
		RK4Integrator integrator;

		const State initialState{
			1.0,
			2.0,
			3.0
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

		runnerA.reset(
			initialState,
			5000);

		runnerB.reset(
			initialState,
			5000);

		runnerA.runSteps(10000);
		runnerB.runSteps(10000);

		expectStateBitwiseEqual(
			runnerA.getState(),
			runnerB.getState());

		EXPECT_EQ(
			runnerA.getCurrentStep(),
			15000U);

		EXPECT_EQ(
			runnerB.getCurrentStep(),
			15000U);

		EXPECT_DOUBLE_EQ(
			runnerA.getCurrentTime(),
			runnerB.getCurrentTime());
	}

	TEST(
		DeterminismTests,
		LongRK4RunsRemainRepeatable)
	{
		DeterministicModel model;
		RK4Integrator integrator;

		const State initialState{
			1.0,
			0.0,
			10.0
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

		runnerA.runSteps(100000);
		runnerB.runSteps(100000);

		expectStateBitwiseEqual(
			runnerA.getState(),
			runnerB.getState());

		EXPECT_TRUE(
			runnerA.getState().isFinite());

		EXPECT_TRUE(
			runnerB.getState().isFinite());
	}

	TEST(
		DeterminismTests,
		ClockTimeDependsOnlyOnStepCountAndTimeStep)
	{
		DeterministicModel model;
		RK4Integrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				1.0,
				0.0,
				1.0
			},
			0.0025);

		runner.runSteps(12345);

		const double expectedTime =
			static_cast<double>(
				runner.getCurrentStep())
			* runner.getClock().getTimeStep();

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			expectedTime);
	}
}