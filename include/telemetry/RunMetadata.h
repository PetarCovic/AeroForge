#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace aeroforge::telemetry
{
	class RunMetadata
	{
	public:
		RunMetadata(
			const std::string& runId,
			std::uint32_t baseSeed,
			std::size_t trialIndex,
			std::uint32_t trialSeed,
			double dtS,
			const std::map<std::string, double>& sampledParameters
		);

		const std::string& getRunId() const;

		std::uint32_t getBaseSeed() const;

		std::size_t getTrialIndex() const;

		std::uint32_t getTrialSeed() const;

		double getDtS() const;
			
		const std::map<std::string, double>& getSampledParameters() const;

	private:
		std::string runId;
		std::uint32_t baseSeed;
		std::size_t trialIndex;
		std::uint32_t trialSeed;
		double dtS;
		std::map<std::string, double> sampledParameters;
	};
}