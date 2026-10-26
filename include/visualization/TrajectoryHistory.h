#pragma once

#include <math/Vector3.h>

#include <vector>
#include <cstddef>

namespace aeroforge::visualization
{
	struct TrajectoryPoint
	{
		double simulationTimeS;
		math::Vector3 positionN;
	};

	class TrajectoryHistory
	{
	public:
		explicit TrajectoryHistory(std::size_t capacity);

		void add(double simulationTimeS, const math::Vector3& positionN);

		void clear();

		bool empty() const;

		std::size_t size() const;

		std::size_t capacity() const;

		const std::vector<TrajectoryPoint>& getPoints() const;

	private:
		std::vector<TrajectoryPoint> points;
		std::size_t maximumCapacity;
	};
}