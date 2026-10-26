#include <visualization/TrajectoryHistory.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::visualization
{
	TrajectoryHistory::TrajectoryHistory(std::size_t capacity)
		: maximumCapacity(capacity)
	{
		if (capacity==0)
		{
			throw std::invalid_argument("capacity cannot equal 0");
		}
	}

	void TrajectoryHistory::add(double simulationTimeS, const math::Vector3& positionN)
	{
		if (!std::isfinite(simulationTimeS) || simulationTimeS < 0)
		{
			throw std::invalid_argument("simulationTimeS must be finite and non-negative");
		}

		if (!positionN.isFinite())
		{
			throw std::invalid_argument("positionN must be finite");
		}

		if (points.size() == maximumCapacity)
		{
			points.erase(points.begin());
		}

		TrajectoryPoint point;
		point.simulationTimeS = simulationTimeS;
		point.positionN = positionN;

		points.push_back(point);
	}

	void TrajectoryHistory::clear()
	{
		points.clear();
	}

	bool TrajectoryHistory::empty() const
	{
		return points.empty();
	}

	std::size_t TrajectoryHistory::size() const
	{
		return points.size();
	}

	std::size_t TrajectoryHistory::capacity() const
	{
		return maximumCapacity;
	}

	const std::vector<TrajectoryPoint>& TrajectoryHistory::getPoints() const
	{
		return points;
	}
}