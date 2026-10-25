#include <gtest/gtest.h>

#include <dynamics/ConstantGravity.h>
#include <dynamics/ForceAccumulator.h>
#include <dynamics/MassProperties.h>
#include <dynamics/TranslationalDynamicsModel.h>
#include <dynamics/TranslationalState.h>

#include <math/Vector3.h>

#include <simulation/RK4Integrator.h>
#include <simulation/SimulationRunner.h>

#include <cmath>

namespace aeroforge::integration::tests
{
	namespace
	{
		constexpr double G = 9.80665;

		double analyticalPosition(
			double initialPosition,
			double initialVelocity,
			double acceleration,
			double time)
		{
			return
				initialPosition
				+ initialVelocity * time
				+ 0.5 * acceleration * time * time;
		}

		double analyticalVelocity(
			double initialVelocity,
			double acceleration,
			double time)
		{
			return
				initialVelocity
				+ acceleration * time;
		}
	}

	TEST(
		BallisticAnalyticalTests,
		RK4MatchesAnalyticalPositionAfterOneSecond)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity(G);

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

		runner.runSteps(100);

		const dynamics::TranslationalState result(
			runner.getState());

		const math::Vector3& position =
			result.getPosition();

		EXPECT_NEAR(
			position.x,
			analyticalPosition(
				0.0,
				30.0,
				0.0,
				1.0),
			1e-10);

		EXPECT_NEAR(
			position.y,
			analyticalPosition(
				0.0,
				10.0,
				0.0,
				1.0),
			1e-10);

		EXPECT_NEAR(
			position.z,
			analyticalPosition(
				0.0,
				-50.0,
				G,
				1.0),
			1e-10);
	}

	TEST(
		BallisticAnalyticalTests,
		RK4MatchesAnalyticalVelocityAfterOneSecond)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity(G);

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

		runner.runSteps(100);

		const dynamics::TranslationalState result(
			runner.getState());

		const math::Vector3& velocity =
			result.getVelocity();

		EXPECT_NEAR(
			velocity.x,
			analyticalVelocity(
				30.0,
				0.0,
				1.0),
			1e-10);

		EXPECT_NEAR(
			velocity.y,
			analyticalVelocity(
				10.0,
				0.0,
				1.0),
			1e-10);

		EXPECT_NEAR(
			velocity.z,
			analyticalVelocity(
				-50.0,
				G,
				1.0),
			1e-10);
	}

	TEST(
		BallisticAnalyticalTests,
		HorizontalVelocityRemainsConstant)
	{
		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity(G);

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

		runner.runSteps(1000);

		const dynamics::TranslationalState result(
			runner.getState());

		EXPECT_NEAR(
			result.getVelocity().x,
			30.0,
			1e-12);

		EXPECT_NEAR(
			result.getVelocity().y,
			10.0,
			1e-12);
	}

	TEST(
		BallisticAnalyticalTests,
		ReachesExpectedApex)
	{
		constexpr double initialUpwardSpeed =
			50.0;

		const double apexTime =
			initialUpwardSpeed / G;

		const double dt =
			apexTime / 1000.0;

		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity(G);

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
				-initialUpwardSpeed));

		simulation::SimulationRunner runner(
			model,
			integrator,
			initial.toStateVector(),
			dt);

		runner.runSteps(1000);

		const dynamics::TranslationalState result(
			runner.getState());

		const double expectedMaximumAltitude =
			(initialUpwardSpeed
				* initialUpwardSpeed)
			/ (2.0 * G);

		const double simulatedAltitude =
			-result.getPosition().z;

		EXPECT_NEAR(
			result.getVelocity().z,
			0.0,
			1e-9);

		EXPECT_NEAR(
			simulatedAltitude,
			expectedMaximumAltitude,
			1e-8);
	}

	TEST(
		BallisticAnalyticalTests,
		ReturnsToLaunchAltitudeAtAnalyticalFlightTime)
	{
		constexpr double initialUpwardSpeed =
			50.0;

		const double flightTime =
			2.0 * initialUpwardSpeed / G;

		constexpr std::uint64_t steps =
			2000;

		const double dt =
			flightTime
			/ static_cast<double>(steps);

		dynamics::MassProperties mass(10.0);
		dynamics::ConstantGravity gravity(G);

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
				-initialUpwardSpeed));

		simulation::SimulationRunner runner(
			model,
			integrator,
			initial.toStateVector(),
			dt);

		runner.runSteps(steps);

		const dynamics::TranslationalState result(
			runner.getState());

		EXPECT_NEAR(
			result.getPosition().z,
			0.0,
			1e-8);

		EXPECT_NEAR(
			result.getVelocity().z,
			initialUpwardSpeed,
			1e-9);
	}

	TEST(
		BallisticAnalyticalTests,
		GravityAccelerationIsIndependentOfMass)
	{
		dynamics::ConstantGravity gravity(G);

		dynamics::MassProperties lightMass(1.0);
		dynamics::MassProperties heavyMass(1000.0);

		dynamics::ForceAccumulator lightForces;
		dynamics::ForceAccumulator heavyForces;

		lightForces.addForce(
			gravity.computeForce(lightMass));

		heavyForces.addForce(
			gravity.computeForce(heavyMass));

		dynamics::TranslationalDynamicsModel lightModel(
			lightForces,
			lightMass);

		dynamics::TranslationalDynamicsModel heavyModel(
			heavyForces,
			heavyMass);

		const simulation::State state{
			0.0, 0.0, 0.0,
			0.0, 0.0, 0.0
		};

		const simulation::StateDerivative light =
			lightModel.evaluate(
				0.0,
				state);

		const simulation::StateDerivative heavy =
			heavyModel.evaluate(
				0.0,
				state);

		EXPECT_NEAR(
			light[5],
			G,
			1e-12);

		EXPECT_NEAR(
			heavy[5],
			G,
			1e-12);

		EXPECT_NEAR(
			light[5],
			heavy[5],
			1e-12);
	}
}