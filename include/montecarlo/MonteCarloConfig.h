#pragma once

#include <montecarlo/ParameterDistribution.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace aeroforge::montecarlo
{
	class MonteCarloConfig
	{
	public:
		MonteCarloConfig(std::size_t trialCount, std::uint32_t baseSeed);

		void addParameter(const std::string& name, const ParameterDistribution& distribution);

		std::size_t getTrialCount() const;

		std::uint32_t getBaseSeed() const;

		const std::map<std::string, ParameterDistribution>& getParameterDistributions() const;

	private:
		std::size_t trialCount;
		std::uint32_t baseSeed;
		std::map<std::string, ParameterDistribution> parameterDistributions;
	};
}