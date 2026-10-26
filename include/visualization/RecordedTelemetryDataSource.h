#pragma once

#include <visualization/VisualizationDataSource.h>

#include <vector>

namespace aeroforge::visualization
{
	class RecordedTelemetryDataSource : public VisualizationDataSource
	{
	public:
		explicit RecordedTelemetryDataSource(std::vector<VisualizationSnapshot> snapshots);

		bool empty() const override;

		std::size_t size() const override;

		double getStartTimeS() const override;

		double getEndTimeS() const override;

		VisualizationSnapshot sampleAt(double simulationTimeS) const override;

	private:
		std::vector<VisualizationSnapshot> snapshots;
	};
}