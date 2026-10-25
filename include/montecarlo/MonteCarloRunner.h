#pragma once

#include <montecarlo/MonteCarloConfig.h>
#include <montecarlo/MonteCarloTypes.h>

#include <functional>
#include <vector>

namespace aeroforge::montecarlo
{
	class MonteCarloRunner
	{
	public:
		using TrialFunction = std::function <MonteCarloResult(const MonteCarloTrial&)>;

		MonteCarloRunner(const MonteCarloConfig& config);

		std::vector<MonteCarloResult> run(const TrialFunction& trialFunction) const;

	private:
		MonteCarloConfig config;
	};
}