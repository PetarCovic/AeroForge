#include <rocket/ThrustCurve.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::rocket
{
	ThrustCurve::ThrustCurve(const std::vector<ThrustPoint>& points)
		: points(points)
	{
		if (points.size() < 2)
		{
			throw std::invalid_argument(
				"points must contain at least 2 points");
		}

		for (std::size_t i = 0; i < points.size(); i++)
		{
			if (!std::isfinite(points[i].timeS))
			{
				throw std::invalid_argument(
					"ThrustPoint timeS must be finite");
			}

			if (points[i].timeS < 0.0)
			{
				throw std::invalid_argument(
					"ThrustPoint timeS must be non-negative");
			}

			if (!std::isfinite(points[i].thrustN))
			{
				throw std::invalid_argument(
					"ThrustPoint thrustN must be finite");
			}

			if (points[i].thrustN < 0.0)
			{
				throw std::invalid_argument(
					"ThrustPoint thrustN must be non-negative");
			}

			if (i > 0
				&& points[i].timeS <= points[i - 1].timeS)
			{
				throw std::invalid_argument(
					"ThrustPoint times must be strictly increasing");
			}
		}
	}

	double ThrustCurve::getThrustN(double timeS) const
	{
		if (!std::isfinite(timeS) || timeS < 0.0)
		{
			throw std::invalid_argument(
				"timeS must be finite and non-negative");
		}

		if (timeS < points.front().timeS
			|| timeS > points.back().timeS)
		{
			return 0.0;
		}

		for (std::size_t i = 0; i < points.size() - 1; i++)
		{
			const ThrustPoint& first =
				points[i];

			const ThrustPoint& second =
				points[i + 1];

			if (timeS >= first.timeS
				&& timeS <= second.timeS)
			{
				const double fraction =
					(timeS - first.timeS)
					/ (second.timeS - first.timeS);

				const double thrustN =
					first.thrustN
					+ fraction
					* (second.thrustN - first.thrustN);

				if (!std::isfinite(thrustN))
				{
					throw std::overflow_error(
						"Interpolated thrust must be finite");
				}

				return thrustN;
			}
		}

		throw std::logic_error(
			"Unable to interpolate thrust curve");
	}
}