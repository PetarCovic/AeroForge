#pragma once

#include <rocket/PropellantTank.h>
#include <rocket/RocketEngine.h>

namespace aeroforge::rocket
{
	double consumePropellantForStep(
		PropellantTank& tank,
		const RocketEngine& engine,
		double timeS,
		double dtS,
		double throttle
	);
}