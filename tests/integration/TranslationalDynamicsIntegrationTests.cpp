#include <gtest/gtest.h>

#include <dynamics/ConstantGravity.h>
#include <dynamics/ForceAccumulator.h>
#include <dynamics/MassProperties.h>
#include <dynamics/TranslationalDynamicsModel.h>
#include <dynamics/TranslationalState.h>

#include <math/Vector3.h>

#include <simulation/EulerIntegrator.h>
#include <simulation/RK4Integrator.h>
#include <simulation/SimulationRunner.h>

#include <cmath>
#include <cstdint>

namespace aeroforge::integration::tests
{
	TEST(
		TranslationalDynamicsIntegrationTests,
		GravityFlowsThroughFullSimulationStack)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity;

		dynamics::ForceAccumulator forces;

		forces.addForce(
			gravity.computeForce(mass));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		simulation::RK4Integrator integrator;

		const dynamics::TranslationalState initial(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				0.0));

		simulation::SimulationRunner runner(
			model,
			integrator,
			initial.toStateVector(),
			0.01);

		runner.runSteps(100);

		const dynamics::TranslationalState result(
			runner.getState());

		EXPECT_NEAR(
			result.getVelocity().z,
			9.80665,
			1e-10);

		EXPECT_NEAR(
			result.getPosition().z,
			4.903325,
			1e-10);
	}

	TEST(
		TranslationalDynamicsIntegrationTests,
		ProjectileInitiallyClimbsThenFalls)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity;

		dynamics::ForceAccumulator forces;

		forces.addForce(
			gravity.computeForce(mass));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		simulation::RK4Integrator integrator;

		const dynamics::TranslationalState initial(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				-50.0));

		simulation::SimulationRunner runner(
			model,
			integrator,
			initial.toStateVector(),
			0.01);

		runner.runSteps(200);

		const dynamics::TranslationalState climbing(
			runner.getState());

		EXPECT_LT(
			climbing.getPosition().z,
			0.0);

		EXPECT_LT(
			climbing.getVelocity().z,
			0.0);

		runner.runSteps(400);

		const dynamics::TranslationalState falling(
			runner.getState());

		EXPECT_GT(
			falling.getVelocity().z,
			0.0);
	}

	TEST(
		TranslationalDynamicsIntegrationTests,
		HorizontalAndVerticalMotionEvolveTogether)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity;

		dynamics::ForceAccumulator forces;

		forces.addForce(
			gravity.computeForce(mass));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		simulation::RK4Integrator integrator;

		const dynamics::TranslationalState initial(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				30.0,
				10.0,
				-50.0));

		simulation::SimulationRunner runner(
			model,
			integrator,
			initial.toStateVector(),
			0.01);

		runner.runSteps(500);

		const dynamics::TranslationalState result(
			runner.getState());

		EXPECT_NEAR(
			result.getPosition().x,
			150.0,
			1e-9);

		EXPECT_NEAR(
			result.getPosition().y,
			50.0,
			1e-9);

		EXPECT_NEAR(
			result.getPosition().z,
			-127.416875,
			1e-8);
	}

	TEST(
		TranslationalDynamicsIntegrationTests,
		AdditionalForceCombinesWithGravity)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity;

		dynamics::ForceAccumulator forces;

		forces.addForce(
			gravity.computeForce(mass));

		forces.addForce(
			math::Vector3(
				100.0,
				50.0,
				-200.0));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		const simulation::State initial{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative derivative =
			model.evaluate(
				0.0,
				initial);

		EXPECT_NEAR(
			derivative[3],
			10.0,
			1e-12);

		EXPECT_NEAR(
			derivative[4],
			5.0,
			1e-12);

		EXPECT_NEAR(
			derivative[5],
			-10.19335,
			1e-12);
	}

	TEST(
		TranslationalDynamicsIntegrationTests,
		ClearingForceAccumulatorRemovesGravityUntilReapplied)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity;

		dynamics::ForceAccumulator forces;

		forces.addForce(
			gravity.computeForce(mass));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		const simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative withGravity =
			model.evaluate(
				0.0,
				state);

		EXPECT_NEAR(
			withGravity[5],
			9.80665,
			1e-12);

		forces.clear();

		const simulation::StateDerivative withoutGravity =
			model.evaluate(
				0.0,
				state);

		EXPECT_DOUBLE_EQ(
			withoutGravity[5],
			0.0);

		forces.addForce(
			gravity.computeForce(mass));

		const simulation::StateDerivative reapplied =
			model.evaluate(
				0.0,
				state);

		EXPECT_NEAR(
			reapplied[5],
			9.80665,
			1e-12);
	}

	TEST(
		TranslationalDynamicsIntegrationTests,
		ChangingMassChangesNonGravityAcceleration)
	{
		dynamics::MassProperties mass(10.0);

		dynamics::ForceAccumulator forces;

		forces.addForce(
			math::Vector3(
				100.0,
				0.0,
				0.0));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		const simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative before =
			model.evaluate(
				0.0,
				state);

		mass.setMassKg(20.0);

		const simulation::StateDerivative after =
			model.evaluate(
				0.0,
				state);

		EXPECT_DOUBLE_EQ(
			before[3],
			10.0);

		EXPECT_DOUBLE_EQ(
			after[3],
			5.0);
	}

	TEST(
		TranslationalDynamicsIntegrationTests,
		EulerAndRK4ProduceSimilarShortDurationTrajectory)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity;

		dynamics::ForceAccumulator forces;

		forces.addForce(
			gravity.computeForce(mass));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		simulation::EulerIntegrator euler;
		simulation::RK4Integrator rk4;

		const dynamics::TranslationalState initial(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				30.0,
				10.0,
				-50.0));

		simulation::SimulationRunner eulerRunner(
			model,
			euler,
			initial.toStateVector(),
			0.001);

		simulation::SimulationRunner rk4Runner(
			model,
			rk4,
			initial.toStateVector(),
			0.001);

		eulerRunner.runSteps(1000);
		rk4Runner.runSteps(1000);

		const dynamics::TranslationalState eulerResult(
			eulerRunner.getState());

		const dynamics::TranslationalState rk4Result(
			rk4Runner.getState());

		EXPECT_NEAR(
			eulerResult.getPosition().x,
			rk4Result.getPosition().x,
			1e-9);

		EXPECT_NEAR(
			eulerResult.getPosition().y,
			rk4Result.getPosition().y,
			1e-9);

		EXPECT_NEAR(
			eulerResult.getVelocity().z,
			rk4Result.getVelocity().z,
			1e-9);

		// Euler integrates position using the velocity at
		// the beginning of each step, so some vertical
		// position error is expected.
		EXPECT_NEAR(
			eulerResult.getPosition().z,
			rk4Result.getPosition().z,
			0.01);
	}

	TEST(
		TranslationalDynamicsIntegrationTests,
		LongBallisticRunRemainsFinite)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity;

		dynamics::ForceAccumulator forces;

		forces.addForce(
			gravity.computeForce(mass));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		simulation::RK4Integrator integrator;

		const dynamics::TranslationalState initial(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				30.0,
				10.0,
				-50.0));

		simulation::SimulationRunner runner(
			model,
			integrator,
			initial.toStateVector(),
			0.01);

		runner.runSteps(100000);

		EXPECT_TRUE(
			runner.getState().isFinite());

		EXPECT_EQ(
			runner.getCurrentStep(),
			100000);

		EXPECT_NEAR(
			runner.getCurrentTime(),
			1000.0,
			1e-12);
	}

	TEST(
		TranslationalDynamicsIntegrationTests,
		ResetReproducesSameBallisticTrajectory)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity;

		dynamics::ForceAccumulator forces;

		forces.addForce(
			gravity.computeForce(mass));

		dynamics::TranslationalDynamicsModel model(
			forces,
			mass);

		simulation::RK4Integrator integrator;

		const dynamics::TranslationalState initial(
			math::Vector3(
				0.0,
				0.0,
				0.0),
			math::Vector3(
				30.0,
				10.0,
				-50.0));

		const simulation::State initialState =
			initial.toStateVector();

		simulation::SimulationRunner runner(
			model,
			integrator,
			initialState,
			0.01);

		runner.runSteps(1000);

		const simulation::State first =
			runner.getState();

		runner.reset(initialState);

		runner.runSteps(1000);

		const simulation::State second =
			runner.getState();

		EXPECT_TRUE(
			first.isApprox(
				second,
				1e-12));
	}
}