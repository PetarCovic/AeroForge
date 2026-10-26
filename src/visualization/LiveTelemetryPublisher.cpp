#include <visualization/LiveTelemetryPublisher.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::visualization
{
	LiveTelemetryPublisher::LiveTelemetryPublisher()
	{

	}

	void LiveTelemetryPublisher::publish(const VisualizationSnapshot& snapshot)
	{
		if (!std::isfinite(snapshot.simulationTimeS) || snapshot.simulationTimeS < 0)
		{
			throw std::invalid_argument("snapshot.simulationTimeS must be finite and non-negative");
		}

		std::unique_lock<std::mutex> lock(mutex, std::try_to_lock);

		if (!lock.owns_lock())
		{
			return;
		}
		latestSnapshot = snapshot;
	}

	bool LiveTelemetryPublisher::hasSnapshot() const
	{
		std::lock_guard<std::mutex> lock(mutex);

		return latestSnapshot.has_value();
	}

	std::optional<VisualizationSnapshot> LiveTelemetryPublisher::getLatestSnapshot() const
	{
		std::lock_guard<std::mutex> lock(mutex);

		return latestSnapshot;
	}
}