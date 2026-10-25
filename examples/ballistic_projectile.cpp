#include <dynamics/ConstantGravity.h>
#include <dynamics/ForceAccumulator.h>
#include <dynamics/MassProperties.h>
#include <dynamics/TranslationalDynamicsModel.h>
#include <dynamics/TranslationalState.h>

#include <math/Vector3.h>

#include <simulation/RK4Integrator.h>
#include <simulation/SimulationRunner.h>

#include <iomanip>
#include <iostream>

int main()
{
	using namespace aeroforge;

	constexpr double massKg = 10.0;
	constexpr double timeStep = 0.01;
	constexpr double durationSeconds = 11.0;

	dynamics::MassProperties mass(massKg);
	dynamics::ConstantGravity gravity;

	dynamics::ForceAccumulator forces;
	forces.addForce(
		gravity.computeForce(mass));

	dynamics::TranslationalDynamicsModel model(
		forces,
		mass);

	simulation::RK4Integrator integrator;

	const math::Vector3 initialPositionN(
		0.0,
		0.0,
		0.0);

	const math::Vector3 initialVelocityN(
		30.0,
		10.0,
		-50.0);

	const dynamics::TranslationalState initialTranslationalState(
		initialPositionN,
		initialVelocityN);

	const simulation::State initialState =
		initialTranslationalState.toStateVector();

	simulation::SimulationRunner runner(
		model,
		integrator,
		initialState,
		timeStep);

	const std::uint64_t totalSteps =
		static_cast<std::uint64_t>(
			durationSeconds / timeStep);

	const std::uint64_t printEvery =
		static_cast<std::uint64_t>(
			1.0 / timeStep);

	std::cout
		<< std::fixed
		<< std::setprecision(3);

	std::cout
		<< "Time(s)"
		<< "\tNorth(m)"
		<< "\tEast(m)"
		<< "\tAltitude(m)"
		<< "\tVN(m/s)"
		<< "\tVE(m/s)"
		<< "\tVD(m/s)"
		<< '\n';

	for (std::uint64_t step = 0;
		step <= totalSteps;
		++step)
	{
		if (step % printEvery == 0)
		{
			const dynamics::TranslationalState state(
				runner.getState());

			const math::Vector3& position =
				state.getPosition();

			const math::Vector3& velocity =
				state.getVelocity();

			const double altitude =
				-position.z;

			std::cout
				<< runner.getCurrentTime()
				<< '\t'
				<< position.x
				<< '\t'
				<< position.y
				<< '\t'
				<< altitude
				<< '\t'
				<< velocity.x
				<< '\t'
				<< velocity.y
				<< '\t'
				<< velocity.z
				<< '\n';
		}

		if (step < totalSteps)
		{
			runner.step();
		}
	}

	return 0;
}