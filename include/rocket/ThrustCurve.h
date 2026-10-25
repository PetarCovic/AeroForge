#pragma once

#include <vector>

namespace aeroforge::rocket
{
	struct ThrustPoint
	{
		double timeS;
		double thrustN;
	};

	class ThrustCurve
	{
	public:
		ThrustCurve(const std::vector<ThrustPoint>& points);

		double getThrustN(double timeS) const;

	private:
		std::vector<ThrustPoint> points;
	};
}