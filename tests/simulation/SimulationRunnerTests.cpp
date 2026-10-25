#include <gtest/gtest.h>

#include <simulation/DynamicsModel.h>
#include <simulation/EulerIntegrator.h>
#include <simulation/Integrator.h>
#include <simulation/SimulationRunner.h>

#include <cstddef>
#include <cstdint>
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

		class RecordingIntegrator final : public Integrator
		{
		public:
			mutable std::uint64_t callCount = 0;
			mutable double lastTime = 0.0;
			mutable double lastDt = 0.0;
			mutable State lastState;

			State step(
				const DynamicsModel& model,
				double time,
				const State& state,
				double dt) const override
			{
				++callCount;

				lastTime = time;
				lastDt = dt;
				lastState = state;

				const StateDerivative derivative =
					model.evaluate(time, state);

				return state + derivative * dt;
			}
		};

		class ThrowingIntegrator final : public Integrator
		{
		public:
			State step(
				const DynamicsModel& model,
				double time,
				const State& state,
				double dt) const override
			{
				(void)model;
				(void)time;
				(void)state;
				(void)dt;

				throw std::runtime_error(
					"Integrator failure");
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
	// Construction
	// ============================================================

	TEST(
		SimulationRunnerTests,
		StoresInitialState)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0,
				0.0
			});

		EulerIntegrator integrator;

		const State initialState{
			10.0,
			20.0
		};

		SimulationRunner runner(
			model,
			integrator,
			initialState,
			0.01);

		expectStateNear(
			runner.getState(),
			initialState);
	}

	TEST(
		SimulationRunnerTests,
		StartsAtStepZero)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.01);

		EXPECT_EQ(
			runner.getCurrentStep(),
			0U);
	}

	TEST(
		SimulationRunnerTests,
		StartsAtTimeZero)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.01);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			0.0);
	}

	TEST(
		SimulationRunnerTests,
		StoresConfiguredTimeStep)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.025);

		EXPECT_DOUBLE_EQ(
			runner.getClock().getTimeStep(),
			0.025);
	}

	TEST(
		SimulationRunnerTests,
		RejectsNonFiniteInitialState)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0
			});

		EulerIntegrator integrator;

		const State invalidState{
			std::numeric_limits<double>::quiet_NaN()
		};

		EXPECT_THROW(
			SimulationRunner(
				model,
				integrator,
				invalidState,
				0.01),
			std::invalid_argument);
	}

	TEST(
		SimulationRunnerTests,
		RejectsInvalidTimeStep)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0
			});

		EulerIntegrator integrator;

		EXPECT_THROW(
			SimulationRunner(
				model,
				integrator,
				State{
					0.0
				},
				0.0),
			std::invalid_argument);

		EXPECT_THROW(
			SimulationRunner(
				model,
				integrator,
				State{
					0.0
				},
				-0.01),
			std::invalid_argument);
	}

	// ============================================================
	// Single step
	// ============================================================

	TEST(
		SimulationRunnerTests,
		StepAdvancesState)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				2.0,
				-4.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				10.0,
				5.0
			},
			0.5);

		runner.step();

		expectStateNear(
			runner.getState(),
			State{
				11.0,
				3.0
			});
	}

	TEST(
		SimulationRunnerTests,
		StepAdvancesClockExactlyOnce)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.1);

		runner.step();

		EXPECT_EQ(
			runner.getCurrentStep(),
			1U);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			0.1);
	}

	TEST(
		SimulationRunnerTests,
		StepUsesCurrentClockTimeAndTimeStep)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		RecordingIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				5.0
			},
			0.25);

		runner.step();

		EXPECT_EQ(
			integrator.callCount,
			1U);

		EXPECT_DOUBLE_EQ(
			integrator.lastTime,
			0.0);

		EXPECT_DOUBLE_EQ(
			integrator.lastDt,
			0.25);

		expectStateNear(
			integrator.lastState,
			State{
				5.0
			});
	}

	TEST(
		SimulationRunnerTests,
		SecondStepUsesUpdatedClockTime)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		RecordingIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.1);

		runner.step();
		runner.step();

		EXPECT_EQ(
			integrator.callCount,
			2U);

		EXPECT_DOUBLE_EQ(
			integrator.lastTime,
			0.1);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			0.2);
	}

	// ============================================================
	// runSteps
	// ============================================================

	TEST(
		SimulationRunnerTests,
		RunStepsAdvancesRequestedNumberOfSteps)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.1);

		runner.runSteps(100);

		EXPECT_EQ(
			runner.getCurrentStep(),
			100U);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			10.0);
	}

	TEST(
		SimulationRunnerTests,
		RunStepsAdvancesStateCorrectly)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				5.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.01);

		runner.runSteps(1000);

		EXPECT_NEAR(
			runner.getState()[0],
			50.0,
			1e-9);
	}

	TEST(
		SimulationRunnerTests,
		RunStepsZeroDoesNothing)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				5.0
			});

		EulerIntegrator integrator;

		const State initialState{
			10.0
		};

		SimulationRunner runner(
			model,
			integrator,
			initialState,
			0.1);

		runner.runSteps(0);

		expectStateNear(
			runner.getState(),
			initialState);

		EXPECT_EQ(
			runner.getCurrentStep(),
			0U);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			0.0);
	}

	TEST(
		SimulationRunnerTests,
		RunStepsMatchesRepeatedManualSteps)
	{
		StateDependentModel model;
		EulerIntegrator integrator;

		const State initialState{
			1.0
		};

		SimulationRunner batchRunner(
			model,
			integrator,
			initialState,
			0.01);

		SimulationRunner manualRunner(
			model,
			integrator,
			initialState,
			0.01);

		batchRunner.runSteps(100);

		for (int i = 0; i < 100; ++i)
		{
			manualRunner.step();
		}

		expectStateNear(
			batchRunner.getState(),
			manualRunner.getState());

		EXPECT_EQ(
			batchRunner.getCurrentStep(),
			manualRunner.getCurrentStep());

		EXPECT_DOUBLE_EQ(
			batchRunner.getCurrentTime(),
			manualRunner.getCurrentTime());
	}

	// ============================================================
	// Reset
	// ============================================================

	TEST(
		SimulationRunnerTests,
		ResetRestoresNewState)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0,
				1.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0,
				0.0
			},
			0.1);

		runner.runSteps(10);

		const State resetState{
			100.0,
			200.0
		};

		runner.reset(
			resetState);

		expectStateNear(
			runner.getState(),
			resetState);
	}

	TEST(
		SimulationRunnerTests,
		DefaultResetReturnsClockToStepZero)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.1);

		runner.runSteps(50);

		runner.reset(
			State{
				0.0
			});

		EXPECT_EQ(
			runner.getCurrentStep(),
			0U);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			0.0);
	}

	TEST(
		SimulationRunnerTests,
		ResetCanSetCustomStartStep)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.1);

		runner.reset(
			State{
				5.0
			},
			25);

		EXPECT_EQ(
			runner.getCurrentStep(),
			25U);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			2.5);
	}

	TEST(
		SimulationRunnerTests,
		ResetPreservesTimeStep)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				0.0
			},
			0.025);

		runner.runSteps(10);

		runner.reset(
			State{
				100.0
			});

		EXPECT_DOUBLE_EQ(
			runner.getClock().getTimeStep(),
			0.025);
	}

	TEST(
		SimulationRunnerTests,
		RejectsNonFiniteResetState)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				0.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				10.0
			},
			0.1);

		const State invalidState{
			std::numeric_limits<double>::infinity()
		};

		EXPECT_THROW(
			runner.reset(
				invalidState),
			std::invalid_argument);
	}

	TEST(
		SimulationRunnerTests,
		FailedResetDoesNotModifyExistingSimulation)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		EulerIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				5.0
			},
			0.1);

		runner.runSteps(5);

		const State stateBefore =
			runner.getState();

		const std::uint64_t stepBefore =
			runner.getCurrentStep();

		const double timeBefore =
			runner.getCurrentTime();

		EXPECT_THROW(
			runner.reset(
				State{
					std::numeric_limits<double>::quiet_NaN()
				}),
			std::invalid_argument);

		expectStateNear(
			runner.getState(),
			stateBefore);

		EXPECT_EQ(
			runner.getCurrentStep(),
			stepBefore);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			timeBefore);
	}

	// ============================================================
	// Failure behavior
	// ============================================================

	TEST(
		SimulationRunnerTests,
		IntegratorFailureDoesNotAdvanceClock)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		ThrowingIntegrator integrator;

		SimulationRunner runner(
			model,
			integrator,
			State{
				10.0
			},
			0.1);

		EXPECT_THROW(
			runner.step(),
			std::runtime_error);

		EXPECT_EQ(
			runner.getCurrentStep(),
			0U);

		EXPECT_DOUBLE_EQ(
			runner.getCurrentTime(),
			0.0);
	}

	TEST(
		SimulationRunnerTests,
		IntegratorFailureDoesNotModifyState)
	{
		ConstantDerivativeModel model(
			StateDerivative{
				1.0
			});

		ThrowingIntegrator integrator;

		const State initialState{
			10.0
		};

		SimulationRunner runner(
			model,
			integrator,
			initialState,
			0.1);

		EXPECT_THROW(
			runner.step(),
			std::runtime_error);

		expectStateNear(
			runner.getState(),
			initialState);
	}

	// ============================================================
	// Determinism
	// ============================================================

	TEST(
		SimulationRunnerTests,
		IdenticalRunsProduceIdenticalResults)
	{
		StateDependentModel model;
		EulerIntegrator integrator;

		const State initialState{
			1.0,
			-2.0,
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

		runnerA.runSteps(10000);
		runnerB.runSteps(10000);

		expectStateNear(
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