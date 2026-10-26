#pragma once

#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/TrajectoryHistory.h>
#include <visualization/VisualizationSnapshot.h>

#include <cstddef>
#include <memory>
#include <optional>

namespace aeroforge::visualization
{
	class LiveVisualizationModel
	{
	public:
		LiveVisualizationModel(std::shared_ptr<const LiveTelemetryPublisher> publisher, std::size_t capacity);

		void update();

		bool hasSnapshot() const;

		const VisualizationSnapshot& getCurrentSnapshot() const;

		const TrajectoryHistory& getTrajectoryHistory() const;

	private:
		std::shared_ptr<const LiveTelemetryPublisher> publisher;
		std::optional<VisualizationSnapshot> currentSnapshot;
		TrajectoryHistory trajectoryHistory;
	};
}