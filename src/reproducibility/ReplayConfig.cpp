#include <reproducibility/ReplayConfig.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::reproducibility
{
	ReplayConfig::ReplayConfig(
		std::size_t trialIndex,
		std::uint32_t trialSeed,
		double dtS,
		const math::Vector3& initialPositionN,
		const math::Vector3& initialVelocityN,
		const math::Quaternion& initialAttitudeNB,
		const math::Vector3& initialAngularVelocityB,
		const std::map<std::string, double>& sampledParameters
	)
		: trialIndex(trialIndex), 
		trialSeed(trialSeed), 
		dtS(dtS), 
		initialPositionN(initialPositionN), 
		initialVelocityN(initialVelocityN), 
		initialAttitudeNB(initialAttitudeNB),
		initialAngularVelocityB(initialAngularVelocityB),
		sampledParameters(sampledParameters)
	{
		if (!std::isfinite(dtS) || dtS <= 0)
		{
			throw std::invalid_argument("dtS must be finite and positive");
		}

		if (!initialPositionN.isFinite())
		{
			throw std::invalid_argument("initialPositionN must be finite");
		}

		if (!initialVelocityN.isFinite())
		{
			throw std::invalid_argument("initialVelocityN must be finite");
		}

		if (!initialAttitudeNB.isFinite())
		{
			throw std::invalid_argument("initialAttitudeNB must be finite");
		}

		if (initialAttitudeNB.isZero())
		{
			throw std::invalid_argument("initialAttitudeNB must be non-zero");
		}

		this->initialAttitudeNB =initialAttitudeNB.normalized();

		if (!initialAngularVelocityB.isFinite())
		{
			throw std::invalid_argument("initialAngularVelocityB must be finite");
		}

		for (const auto& [key, value] : sampledParameters) 
		{
			if (key.empty())
			{
				throw std::invalid_argument("sampledParameters key cannot be empty");
			}

			if (!std::isfinite(value))
			{
				throw std::invalid_argument("sampledParameters value must be finite");
			}
		}
	}

	std::size_t ReplayConfig::getTrialIndex() const
	{
		return trialIndex;
	}

	std::uint32_t ReplayConfig::getTrialSeed() const
	{
		return trialSeed;
	}

	double ReplayConfig::getDtS() const
	{
		return dtS;
	}

	const math::Vector3& ReplayConfig::getInitialPositionN() const
	{
		return initialPositionN;
	}

	const math::Vector3& ReplayConfig::getInitialVelocityN() const
	{
		return initialVelocityN;
	}

	const math::Quaternion& ReplayConfig::getInitialAttitudeNB() const
	{
		return initialAttitudeNB;
	}

	const math::Vector3& ReplayConfig::getInitialAngularVelocityB() const
	{
		return initialAngularVelocityB;
	}

	const std::map<std::string, double>& ReplayConfig::getSampledParameters() const
	{
		return sampledParameters;
	}
}