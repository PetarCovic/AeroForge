#include <montecarlo/MonteCarloRunner.h>

#include <map>
#include <random>
#include <stdexcept>

namespace aeroforge::montecarlo
{
	MonteCarloRunner::MonteCarloRunner(const MonteCarloConfig& config)
		: config(config)
	{

	}

	std::vector<MonteCarloResult> MonteCarloRunner::run(const TrialFunction& trialFunction) const
	{
		if (!trialFunction)
		{
			throw std::invalid_argument("trialFunction cannot be empty");
		}

		std::vector<MonteCarloResult> results;
		results.reserve(config.getTrialCount());

		for (std::size_t i = 0; i < config.getTrialCount(); ++i)
		{
			std::uint32_t seed = config.getBaseSeed()+ static_cast<std::uint32_t>(i);

			std::mt19937 randomEngine(seed);

			std::map<std::string, double> parameters;

			for (const auto& [name, distribution] : config.getParameterDistributions())
			{
				double sampledValue =distribution.sample(randomEngine);

				parameters.emplace(name, sampledValue);
			}

			MonteCarloTrial trial{i, seed, parameters};

			MonteCarloResult result =trialFunction(trial);

			results.push_back(result);
		}

		return results;
	}
}