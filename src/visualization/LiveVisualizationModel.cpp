#include <visualization/LiveVisualizationModel.h>

#include <stdexcept>

namespace aeroforge::visualization
{
	LiveVisualizationModel::LiveVisualizationModel(
		std::shared_ptr<const LiveTelemetryPublisher> publisher,
		std::size_t capacity)
		: publisher(publisher),
		trajectoryHistory(capacity)
	{
		if (!publisher)
		{
			throw std::invalid_argument(
				"publisher cannot be null");
		}
	}

	void LiveVisualizationModel::update()
	{
		const std::optional<VisualizationSnapshot> latestSnapshot =
			publisher->getLatestSnapshot();

		if (!latestSnapshot.has_value())
		{
			return;
		}

		if (
			currentSnapshot.has_value() &&
			currentSnapshot->simulationTimeS ==
			latestSnapshot->simulationTimeS)
		{
			return;
		}

		if (currentSnapshot.has_value() && latestSnapshot->simulationTimeS < currentSnapshot->simulationTimeS)
		{
			trajectoryHistory.clear();
		}

		currentSnapshot = latestSnapshot;

		trajectoryHistory.add(
			currentSnapshot->simulationTimeS,
			currentSnapshot->truthPositionN);
	}

	bool LiveVisualizationModel::hasSnapshot() const
	{
		return currentSnapshot.has_value();
	}

	const VisualizationSnapshot&
		LiveVisualizationModel::getCurrentSnapshot() const
	{
		if (!currentSnapshot.has_value())
		{
			throw std::logic_error(
				"No snapshot exists yet");
		}

		return *currentSnapshot;
	}

	const TrajectoryHistory&
		LiveVisualizationModel::getTrajectoryHistory() const
	{
		return trajectoryHistory;
	}
}