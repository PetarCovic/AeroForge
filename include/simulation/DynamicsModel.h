#pragma once

#include <simulation/StateDerivative.h>

namespace aeroforge::simulation
{
	class DynamicsModel
	{
	public:
		virtual ~DynamicsModel() = default;

		virtual StateDerivative evaluate(double time, const State& state) const = 0;
	};
}