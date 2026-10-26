#pragma once

#include <visualization/VisualizationSnapshot.h>

#include <optional>
#include <mutex>

namespace aeroforge::visualization
{
	class LiveTelemetryPublisher
	{
	public:
		LiveTelemetryPublisher();

		void publish(const VisualizationSnapshot& snapshot);

		bool hasSnapshot() const;

		std::optional<VisualizationSnapshot> getLatestSnapshot() const;

	private:
		std::optional<VisualizationSnapshot> latestSnapshot;
		mutable std::mutex mutex;
	};
}