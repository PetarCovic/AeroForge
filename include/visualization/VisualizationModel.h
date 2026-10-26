#pragma once

#include <visualization/VisualizationSnapshot.h>
#include <visualization/PlaybackController.h>
#include <visualization/VisualizationDataSource.h>

#include <memory>

namespace aeroforge::visualization
{
	class VisualizationModel
	{
	public:
		explicit VisualizationModel(std::shared_ptr<const VisualizationDataSource> dataSource);

		void play();

		void pause();

		void restart();

		void seek(double simulationTimeS);

		void update(double wallDeltaS);

		void setPlaybackRate(double playbackRate);

		double getCurrentTimeS() const;

		double getPlaybackRate() const;

		bool isPlaying() const;

		const VisualizationSnapshot& getCurrentSnapshot() const;

	private:
		std::shared_ptr<const VisualizationDataSource> dataSource;
		PlaybackController playbackController;
		VisualizationSnapshot currentSnapshot;
	};
}