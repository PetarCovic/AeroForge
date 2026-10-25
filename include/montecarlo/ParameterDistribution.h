#pragma once

#include <random>

namespace aeroforge::montecarlo
{
	enum class DistributionType
	{
		Constant,
		Uniform,
		Normal
	};

	class ParameterDistribution
	{
	public:
		static ParameterDistribution constant(double value);

		static ParameterDistribution uniform(double minValue, double maxValue);

		static ParameterDistribution normal(double mean, double standardDeviation);

		double sample(std::mt19937& randomEngine) const;

	private:
		ParameterDistribution(DistributionType type, double parameterA, double parameterB);

		DistributionType type;

		double parameterA;
		double parameterB;
	};
}