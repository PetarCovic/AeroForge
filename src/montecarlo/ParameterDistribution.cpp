#include <montecarlo/ParameterDistribution.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::montecarlo
{
	ParameterDistribution::ParameterDistribution(DistributionType type, double parameterA, double parameterB)
		: type(type), parameterA(parameterA), parameterB(parameterB)
	{

	}

	ParameterDistribution ParameterDistribution::constant(double value)
	{
		if (!std::isfinite(value))
		{
			throw std::invalid_argument("value must be finite");
		}

		return ParameterDistribution(montecarlo::DistributionType::Constant, value, 0.0);
	}

	ParameterDistribution ParameterDistribution::uniform(double minValue, double maxValue)
	{
		if (!std::isfinite(minValue))
		{
			throw std::invalid_argument("minValue must be finite");
		}

		if (!std::isfinite(maxValue))
		{
			throw std::invalid_argument("maxValue must be finite");
		}

		if (maxValue < minValue)
		{
			throw std::invalid_argument("maxValue must be greater or equal to min value");
		}

		return ParameterDistribution(montecarlo::DistributionType::Uniform, minValue, maxValue);
	}

	ParameterDistribution ParameterDistribution::normal(double mean, double standardDeviation)
	{
		if (!std::isfinite(mean))
		{
			throw std::invalid_argument("mean must be finite");
		}

		if (!std::isfinite(standardDeviation) || standardDeviation<0)
		{
			throw std::invalid_argument("standardDeviation must be finite and non-negative");
		}

		return ParameterDistribution(montecarlo::DistributionType::Normal, mean, standardDeviation);
	}

	double ParameterDistribution::sample(std::mt19937& randomEngine) const
	{
		switch (type)
		{
			case montecarlo::DistributionType::Constant:
			{
				return parameterA;
			}

			case montecarlo::DistributionType::Uniform:
			{
				std::uniform_real_distribution<double> uniform(parameterA, parameterB);
				double uniformOutput=uniform(randomEngine);

				if (!std::isfinite(uniformOutput))
				{
					throw std::overflow_error("uniformOutput must be finite");
				}

				return uniformOutput;
			}
				
			case montecarlo::DistributionType::Normal:
			{
				if (parameterB == 0.0)
				{
					return parameterA;
				}

				std::normal_distribution<double> normal(parameterA, parameterB);
				double normalOutput=normal(randomEngine);

				if (!std::isfinite(normalOutput))
				{
					throw std::overflow_error("normalOutput must be finite");
				}

				return normalOutput;
			}

			default:
			{
				throw std::runtime_error("No valid DistributionType matched");
			}
		}
	}
}