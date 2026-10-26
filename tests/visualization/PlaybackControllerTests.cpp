#include <visualization/PlaybackController.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aeroforge::visualization
{
	TEST(
		PlaybackControllerTests,
		ConstructsAtStartTimeAndPaused)
	{
		const PlaybackController controller(
			5.0,
			20.0);

		EXPECT_DOUBLE_EQ(
			controller.getStartTimeS(),
			5.0);

		EXPECT_DOUBLE_EQ(
			controller.getEndTimeS(),
			20.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			5.0);

		EXPECT_DOUBLE_EQ(
			controller.getPlaybackRate(),
			1.0);

		EXPECT_FALSE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		AllowsZeroDurationInterval)
	{
		const PlaybackController controller(
			5.0,
			5.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			5.0);

		EXPECT_FALSE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		RejectsEndBeforeStart)
	{
		EXPECT_THROW(
			PlaybackController(
				10.0,
				5.0),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		RejectsNonFiniteStartTime)
	{
		EXPECT_THROW(
			PlaybackController(
				std::numeric_limits<double>::infinity(),
				10.0),
			std::invalid_argument);

		EXPECT_THROW(
			PlaybackController(
				-std::numeric_limits<double>::infinity(),
				10.0),
			std::invalid_argument);

		EXPECT_THROW(
			PlaybackController(
				std::numeric_limits<double>::quiet_NaN(),
				10.0),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		RejectsNonFiniteEndTime)
	{
		EXPECT_THROW(
			PlaybackController(
				0.0,
				std::numeric_limits<double>::infinity()),
			std::invalid_argument);

		EXPECT_THROW(
			PlaybackController(
				0.0,
				-std::numeric_limits<double>::infinity()),
			std::invalid_argument);

		EXPECT_THROW(
			PlaybackController(
				0.0,
				std::numeric_limits<double>::quiet_NaN()),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		PlaySetsPlayingState)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();

		EXPECT_TRUE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		PlayAtEndRemainsPaused)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.seek(
			10.0);

		controller.play();

		EXPECT_FALSE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		ZeroDurationIntervalCannotPlay)
	{
		PlaybackController controller(
			5.0,
			5.0);

		controller.play();

		EXPECT_FALSE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		PauseClearsPlayingState)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();
		controller.pause();

		EXPECT_FALSE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		PausedUpdateDoesNotAdvanceTime)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.update(
			1.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			0.0);
	}

	TEST(
		PlaybackControllerTests,
		PlayingUpdateAdvancesAtNormalRate)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();

		controller.update(
			0.5);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			0.5);
	}

	TEST(
		PlaybackControllerTests,
		PlaybackRateScalesTimeAdvance)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.setPlaybackRate(
			2.0);

		controller.play();

		controller.update(
			0.5);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			1.0);
	}

	TEST(
		PlaybackControllerTests,
		PlaybackRateCanBeChangedDuringPlayback)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();

		controller.update(
			1.0);

		controller.setPlaybackRate(
			2.0);

		controller.update(
			1.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			3.0);
	}

	TEST(
		PlaybackControllerTests,
		RejectsZeroPlaybackRate)
	{
		PlaybackController controller(
			0.0,
			10.0);

		EXPECT_THROW(
			controller.setPlaybackRate(
				0.0),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		RejectsNegativePlaybackRate)
	{
		PlaybackController controller(
			0.0,
			10.0);

		EXPECT_THROW(
			controller.setPlaybackRate(
				-1.0),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		RejectsNonFinitePlaybackRate)
	{
		PlaybackController controller(
			0.0,
			10.0);

		EXPECT_THROW(
			controller.setPlaybackRate(
				std::numeric_limits<double>::infinity()),
			std::invalid_argument);

		EXPECT_THROW(
			controller.setPlaybackRate(
				-std::numeric_limits<double>::infinity()),
			std::invalid_argument);

		EXPECT_THROW(
			controller.setPlaybackRate(
				std::numeric_limits<double>::quiet_NaN()),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		SeekMovesCurrentTime)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.seek(
			4.5);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			4.5);
	}

	TEST(
		PlaybackControllerTests,
		SeekBeforeStartClampsToStart)
	{
		PlaybackController controller(
			5.0,
			10.0);

		controller.seek(
			-100.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			5.0);
	}

	TEST(
		PlaybackControllerTests,
		SeekAfterEndClampsToEnd)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.seek(
			100.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			10.0);
	}

	TEST(
		PlaybackControllerTests,
		SeekingToEndPausesPlayback)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();

		controller.seek(
			10.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			10.0);

		EXPECT_FALSE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		RejectsNonFiniteSeekTime)
	{
		PlaybackController controller(
			0.0,
			10.0);

		EXPECT_THROW(
			controller.seek(
				std::numeric_limits<double>::infinity()),
			std::invalid_argument);

		EXPECT_THROW(
			controller.seek(
				-std::numeric_limits<double>::infinity()),
			std::invalid_argument);

		EXPECT_THROW(
			controller.seek(
				std::numeric_limits<double>::quiet_NaN()),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		UpdateClampsAtEnd)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();

		controller.update(
			100.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			10.0);
	}

	TEST(
		PlaybackControllerTests,
		ReachingEndAutomaticallyPauses)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();

		controller.update(
			10.0);

		EXPECT_FALSE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		PartialAdvanceBeforeEndRemainsPlaying)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();

		controller.update(
			9.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			9.0);

		EXPECT_TRUE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		RestartReturnsToStartAndPauses)
	{
		PlaybackController controller(
			5.0,
			20.0);

		controller.play();

		controller.update(
			5.0);

		controller.restart();

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			5.0);

		EXPECT_FALSE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		RestartPreservesPlaybackRate)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.setPlaybackRate(
			2.0);

		controller.play();

		controller.update(
			1.0);

		controller.restart();

		EXPECT_DOUBLE_EQ(
			controller.getPlaybackRate(),
			2.0);
	}

	TEST(
		PlaybackControllerTests,
		ZeroUpdateDoesNotAdvanceTime)
	{
		PlaybackController controller(
			0.0,
			10.0);

		controller.play();

		controller.update(
			0.0);

		EXPECT_DOUBLE_EQ(
			controller.getCurrentTimeS(),
			0.0);

		EXPECT_TRUE(
			controller.isPlaying());
	}

	TEST(
		PlaybackControllerTests,
		RejectsNegativeUpdateDelta)
	{
		PlaybackController controller(
			0.0,
			10.0);

		EXPECT_THROW(
			controller.update(
				-0.1),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		RejectsNonFiniteUpdateDelta)
	{
		PlaybackController controller(
			0.0,
			10.0);

		EXPECT_THROW(
			controller.update(
				std::numeric_limits<double>::infinity()),
			std::invalid_argument);

		EXPECT_THROW(
			controller.update(
				-std::numeric_limits<double>::infinity()),
			std::invalid_argument);

		EXPECT_THROW(
			controller.update(
				std::numeric_limits<double>::quiet_NaN()),
			std::invalid_argument);
	}

	TEST(
		PlaybackControllerTests,
		RejectsOverflowingPlaybackAdvance)
	{
		PlaybackController controller(
			0.0,
			std::numeric_limits<double>::max());

		controller.setPlaybackRate(
			std::numeric_limits<double>::max());

		controller.play();

		EXPECT_THROW(
			controller.update(
				2.0),
			std::overflow_error);
	}
}