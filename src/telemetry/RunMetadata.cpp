#include <telemetry/RunMetadata.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::telemetry
{
	RunMetadata::RunMetadata(
		const std::string& runId,
		std::uint32_t baseSeed,
		std::size_t trialIndex,
		std::uint32_t trialSeed,
		double dtS,
		const std::map<std::string, double>& sampledParameters
	)
		: runId(runId), baseSeed(baseSeed), trialIndex(trialIndex), trialSeed(trialSeed), dtS(dtS), sampledParameters(sampledParameters)
	{
		if (runId.empty())
		{
			throw std::invalid_argument("runId cannot be empty");
		}

		if (!std::isfinite(dtS) || dtS <= 0)
		{
			throw std::invalid_argument("dtS must be finite and positive");
		}

		for (const auto& [key, value] : sampledParameters)
		{
			if (key.empty())
			{
				throw std::invalid_argument("key cannot be empty");
			}

			if (!std::isfinite(value))
			{
				throw std::invalid_argument("value must be finite");
			}
		}
	}

	const std::string& RunMetadata::getRunId() const
	{
		return runId;
	}

	std::uint32_t RunMetadata::getBaseSeed() const
	{
		return baseSeed;
	}

	std::size_t RunMetadata::getTrialIndex() const
	{
		return trialIndex;
	}

	std::uint32_t RunMetadata::getTrialSeed() const
	{
		return trialSeed;
	}

	double RunMetadata::getDtS() const
	{
		return dtS;
	}

	const std::map<std::string, double>& RunMetadata::getSampledParameters() const
	{
		return sampledParameters;
	}
}