#include <visualization/RecordedTelemetryDataSource.h>
#include <visualization/VisualizationInterpolator.h>

#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <iterator>
#include<cstddef>

namespace aeroforge::visualization
{
	RecordedTelemetryDataSource::RecordedTelemetryDataSource(std::vector<VisualizationSnapshot> snapshots)
		: snapshots(snapshots)
	{
		if (snapshots.empty())
		{
			throw std::invalid_argument("snapshots cannot be empty");
		}

		if (!std::isfinite(snapshots.front().simulationTimeS) || snapshots.front().simulationTimeS<0)
		{
			throw std::invalid_argument("snapshots[0].simulationTimeS must be finite and non-negative");
		}

		double timeS = snapshots.front().simulationTimeS;

		for (std::size_t i=1; i<snapshots.size(); i++)
		{
			if (!std::isfinite(snapshots[i].simulationTimeS))
			{
				throw std::invalid_argument("snapshot.simulationTimeS must be finite");
			}

			if (snapshots[i].simulationTimeS <= timeS)
			{
				throw std::invalid_argument("snapshot.simulationTimeS must be greater than previous time");
			}
			timeS = snapshots[i].simulationTimeS;
		}
	}

	bool RecordedTelemetryDataSource::empty() const
	{
		return snapshots.empty();
	}

	std::size_t RecordedTelemetryDataSource::size() const
	{
		return snapshots.size();
	}

	double RecordedTelemetryDataSource::getStartTimeS() const
	{
		return snapshots.front().simulationTimeS;
	}

	double RecordedTelemetryDataSource::getEndTimeS() const
	{
		return snapshots.back().simulationTimeS;
	}

	VisualizationSnapshot RecordedTelemetryDataSource::sampleAt(double simulationTimeS) const
	{
		if (!std::isfinite(simulationTimeS))
		{
			throw std::invalid_argument("simulationTimeS must be finite");
		}

		if (simulationTimeS <= snapshots.front().simulationTimeS)
		{
			return snapshots.front();
		}

		if (simulationTimeS >= snapshots.back().simulationTimeS)
		{
			return snapshots.back();
		}

		const auto upper =
			std::lower_bound(
				snapshots.begin(),
				snapshots.end(),
				simulationTimeS,
				[](
					const VisualizationSnapshot& snapshot,
					double timeS)
				{
					return snapshot.simulationTimeS < timeS;
				});

		if (upper->simulationTimeS == simulationTimeS)
		{
			return *upper;
		}

		const auto lower =
			std::prev(upper);

		return VisualizationInterpolator::interpolate(*lower, *upper, simulationTimeS);
	}
}