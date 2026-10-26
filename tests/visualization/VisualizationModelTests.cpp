#include <visualization/VisualizationModel.h>
#include <visualization/VisualizationDataSource.h>
#include <visualization/VisualizationSnapshot.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <stdexcept>

namespace aeroforge::visualization
{
	namespace
	{
		VisualizationSnapshot makeModelSnapshot(
			double timeS)
		{
			const math::Vector3 vector(
				timeS,
				timeS,
				timeS);

			return VisualizationSnapshot{
				timeS,

				vector,
				vector,
				math::Quaternion::identity(),
				vector,

				vector,
				vector,
				math::Quaternion::identity(),
				vector,

				vector,
				vector,
				vector
			};
		}

		class FakeVisualizationDataSource
			: public VisualizationDataSource
		{
		public:
			FakeVisualizationDataSource(
				double startTimeS,
				double endTimeS)
				: startTimeS(startTimeS),
				endTimeS(endTimeS)
			{
			}

			bool empty() const override
			{
				return false;
			}

			std::size_t size() const override
			{
				return 2;
			}

			double getStartTimeS() const override
			{
				return startTimeS;
			}

			double getEndTimeS() const override
			{
				return endTimeS;
			}

			VisualizationSnapshot sampleAt(
				double simulationTimeS) const override
			{
				const double clampedTimeS =
					std::clamp(
						simulationTimeS,
						startTimeS,
						endTimeS);

				return makeModelSnapshot(
					clampedTimeS);
			}

		private:
			double startTimeS;
			double endTimeS;
		};

		class EmptyVisualizationDataSource
			: public VisualizationDataSource
		{
		public:
			bool empty() const override
			{
				return true;
			}

			std::size_t size() const override
			{
				return 0;
			}

			double getStartTimeS() const override
			{
				return 0.0;
			}

			double getEndTimeS() const override
			{
				return 0.0;
			}

			VisualizationSnapshot sampleAt(
				double) const override
			{
				throw std::logic_error(
					"Empty data source cannot be sampled");
			}
		};
	}

	TEST(
		VisualizationModelTests,
		ConstructsAtBeginningOfDataSource)
	{
		const auto source =
			std::make_shared<FakeVisualizationDataSource>(
				5.0,
				20.0);

		const VisualizationModel model(
			source);

		EXPECT_DOUBLE_EQ(
			model.getCurrentTimeS(),
			5.0);

		EXPECT_FALSE(
			model.isPlaying());

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			5.0);
	}

	TEST(
		VisualizationModelTests,
		RejectsNullDataSource)
	{
		EXPECT_THROW(
			VisualizationModel(
				nullptr),
			std::invalid_argument);
	}

	TEST(
		VisualizationModelTests,
		RejectsEmptyDataSource)
	{
		const auto source =
			std::make_shared<EmptyVisualizationDataSource>();

		EXPECT_THROW(
			(void)VisualizationModel(source),
			std::invalid_argument);
	}

	TEST(
		VisualizationModelTests,
		PlayAndUpdateAdvanceCurrentSnapshot)
	{
		const auto source =
			std::make_shared<FakeVisualizationDataSource>(
				0.0,
				10.0);

		VisualizationModel model(
			source);

		model.play();

		model.update(
			2.0);

		EXPECT_DOUBLE_EQ(
			model.getCurrentTimeS(),
			2.0);

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			2.0);

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().truthPositionN.x,
			2.0);
	}

	TEST(
		VisualizationModelTests,
		PauseStopsPlaybackAdvance)
	{
		const auto source =
			std::make_shared<FakeVisualizationDataSource>(
				0.0,
				10.0);

		VisualizationModel model(
			source);

		model.play();

		model.update(
			2.0);

		model.pause();

		model.update(
			5.0);

		EXPECT_DOUBLE_EQ(
			model.getCurrentTimeS(),
			2.0);
	}

	TEST(
		VisualizationModelTests,
		SeekUpdatesCurrentSnapshot)
	{
		const auto source =
			std::make_shared<FakeVisualizationDataSource>(
				0.0,
				10.0);

		VisualizationModel model(
			source);

		model.seek(
			7.5);

		EXPECT_DOUBLE_EQ(
			model.getCurrentTimeS(),
			7.5);

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			7.5);

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().truthPositionN.x,
			7.5);
	}

	TEST(
		VisualizationModelTests,
		PlaybackRateControlsAdvance)
	{
		const auto source =
			std::make_shared<FakeVisualizationDataSource>(
				0.0,
				10.0);

		VisualizationModel model(
			source);

		model.setPlaybackRate(
			2.0);

		model.play();

		model.update(
			1.5);

		EXPECT_DOUBLE_EQ(
			model.getCurrentTimeS(),
			3.0);

		EXPECT_DOUBLE_EQ(
			model.getPlaybackRate(),
			2.0);
	}

	TEST(
		VisualizationModelTests,
		RestartReturnsToFirstSnapshot)
	{
		const auto source =
			std::make_shared<FakeVisualizationDataSource>(
				5.0,
				20.0);

		VisualizationModel model(
			source);

		model.play();

		model.update(
			5.0);

		model.restart();

		EXPECT_DOUBLE_EQ(
			model.getCurrentTimeS(),
			5.0);

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			5.0);

		EXPECT_FALSE(
			model.isPlaying());
	}

	TEST(
		VisualizationModelTests,
		ReachingEndStopsPlayback)
	{
		const auto source =
			std::make_shared<FakeVisualizationDataSource>(
				0.0,
				10.0);

		VisualizationModel model(
			source);

		model.play();

		model.update(
			100.0);

		EXPECT_DOUBLE_EQ(
			model.getCurrentTimeS(),
			10.0);

		EXPECT_FALSE(
			model.isPlaying());

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			10.0);
	}
}