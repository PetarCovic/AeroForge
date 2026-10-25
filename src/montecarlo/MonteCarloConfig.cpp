#include <montecarlo/MonteCarloConfig.h>

#include <stdexcept>

namespace aeroforge::montecarlo
{
	MonteCarloConfig::MonteCarloConfig(std::size_t trialCount, std::uint32_t baseSeed)
		: trialCount(trialCount), baseSeed(baseSeed)
	{
		if (trialCount == 0)
		{
			throw std::invalid_argument("trialCount cannot equal 0");
		}
	}

	void MonteCarloConfig::addParameter(const std::string& name, const ParameterDistribution& distribution)
	{
		if (name.empty())
		{
			throw std::invalid_argument("name cannot be empty");
		}

		if (parameterDistributions.contains(name))
		{
			throw std::invalid_argument("name must be unique");
		}

		parameterDistributions.emplace(name, distribution);
	}

	std::size_t MonteCarloConfig::getTrialCount() const
	{
		return trialCount;
	}

	std::uint32_t MonteCarloConfig::getBaseSeed() const
	{
		return baseSeed;
	}

	const std::map<std::string, ParameterDistribution>& MonteCarloConfig::getParameterDistributions() const
	{
		return parameterDistributions;
	}
}