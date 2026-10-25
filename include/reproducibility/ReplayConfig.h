#pragma once

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace aeroforge::reproducibility
{
	class ReplayConfig
	{
	public:
		ReplayConfig(
			std::size_t trialIndex,
			std::uint32_t trialSeed,
			double dtS,
			const math::Vector3& initialPositionN,
			const math::Vector3& initialVelocityN,
			const math::Quaternion& initialAttitudeNB,
			const math::Vector3& initialAngularVelocityB,
			const std::map<std::string, double>& sampledParameters
		);

		std::size_t getTrialIndex() const;

		std::uint32_t getTrialSeed() const;

		double getDtS() const;

		const math::Vector3& getInitialPositionN() const;

		const math::Vector3& getInitialVelocityN() const;

		const math::Quaternion& getInitialAttitudeNB() const;

		const math::Vector3& getInitialAngularVelocityB() const;

		const std::map<std::string, double>& getSampledParameters() const;

	private:
		std::size_t trialIndex;
		std::uint32_t trialSeed;
		double dtS;

		math::Vector3 initialPositionN;
		math::Vector3 initialVelocityN;
		math::Quaternion initialAttitudeNB;
		math::Vector3 initialAngularVelocityB;

		std::map<std::string, double> sampledParameters;
	};
}