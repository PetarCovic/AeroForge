#include <visualization/VisualizationModel.h>

#include <stdexcept>

namespace aeroforge::visualization
{
	namespace
	{
		std::shared_ptr<const VisualizationDataSource>
			validateDataSource(
				std::shared_ptr<const VisualizationDataSource> dataSource)
		{
			if (!dataSource)
			{
				throw std::invalid_argument(
					"dataSource cannot be null");
			}

			if (dataSource->empty())
			{
				throw std::invalid_argument(
					"dataSource cannot be empty");
			}

			return dataSource;
		}
	}

	VisualizationModel::VisualizationModel(std::shared_ptr<const VisualizationDataSource> dataSource)
		: dataSource(
			validateDataSource(dataSource)),
		playbackController(
			this->dataSource->getStartTimeS(),
			this->dataSource->getEndTimeS()),
		currentSnapshot(
			this->dataSource->sampleAt(
				this->dataSource->getStartTimeS()))
	{
	}

	void VisualizationModel::play()
	{
		playbackController.play();
	}

	void VisualizationModel::pause()
	{
		playbackController.pause();
	}

	void VisualizationModel::restart()
	{
		playbackController.restart();

		currentSnapshot =
			dataSource->sampleAt(
				playbackController.getCurrentTimeS());
	}

	void VisualizationModel::seek(
		double simulationTimeS)
	{
		playbackController.seek(
			simulationTimeS);

		currentSnapshot =
			dataSource->sampleAt(
				playbackController.getCurrentTimeS());
	}

	void VisualizationModel::update(
		double wallDeltaS)
	{
		playbackController.update(
			wallDeltaS);

		currentSnapshot =
			dataSource->sampleAt(
				playbackController.getCurrentTimeS());
	}

	void VisualizationModel::setPlaybackRate(
		double playbackRate)
	{
		playbackController.setPlaybackRate(
			playbackRate);
	}

	double VisualizationModel::getCurrentTimeS() const
	{
		return playbackController.getCurrentTimeS();
	}

	double VisualizationModel::getPlaybackRate() const
	{
		return playbackController.getPlaybackRate();
	}

	bool VisualizationModel::isPlaying() const
	{
		return playbackController.isPlaying();
	}

	const VisualizationSnapshot&
		VisualizationModel::getCurrentSnapshot() const
	{
		return currentSnapshot;
	}
}