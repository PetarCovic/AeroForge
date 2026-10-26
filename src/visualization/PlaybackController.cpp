#include <visualization/PlaybackController.h>

#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace aeroforge::visualization
{
	PlaybackController::PlaybackController(double startTimeS, double endTimeS)
		: startTimeS(startTimeS), endTimeS(endTimeS), currentTimeS(startTimeS), playbackRate(1.0), playing(false)
	{
		if (!std::isfinite(startTimeS))
		{
			throw std::invalid_argument("startTimeS must be finite");
		}

		if (!std::isfinite(endTimeS))
		{
			throw std::invalid_argument("endTimeS must be finite");
		}

		if (endTimeS < startTimeS)
		{
			throw std::invalid_argument("endTimeS must not be before startTimeS");
		}
	}

	void PlaybackController::play()
	{
		if (currentTimeS < endTimeS)
		{
			playing = true;
		}
	}

	void PlaybackController::pause()
	{
		playing = false;
	}

	void PlaybackController::setPlaybackRate(double playbackRate)
	{
		if (!std::isfinite(playbackRate) || playbackRate<=0)
		{
			throw std::invalid_argument("playbackRate must be finite and positive");
		}

		this->playbackRate = playbackRate;
	}

	void PlaybackController::seek(double simulationTimeS)
	{
		if (!std::isfinite(simulationTimeS))
		{
			throw std::invalid_argument("simulationTimeS must be finite");
		}

		currentTimeS = std::clamp(simulationTimeS, startTimeS, endTimeS);

		if (currentTimeS >= endTimeS)
		{
			playing = false;
		}
	}

	void PlaybackController::update(double wallDeltaS)
	{
		if (!std::isfinite(wallDeltaS) || wallDeltaS < 0.0)
		{
			throw std::invalid_argument("wallDeltaS must be finite and non-negative");
		}

		if (!playing)
		{
			return;
		}

		const double playbackDeltaS = wallDeltaS * playbackRate;

		if (!std::isfinite(playbackDeltaS))
		{
			throw std::overflow_error("Playback time advance must be finite");
		}

		const double remainingTimeS = endTimeS - currentTimeS;

		if (playbackDeltaS >= remainingTimeS)
		{
			currentTimeS = endTimeS;
			playing = false;
			return;
		}

		currentTimeS += playbackDeltaS;
	}

	void PlaybackController::restart()
	{
		currentTimeS = startTimeS;
		playing = false;
	}

	double PlaybackController::getStartTimeS() const
	{
		return startTimeS;
	}

	double PlaybackController::getEndTimeS() const
	{
		return endTimeS;
	}

	double PlaybackController::getCurrentTimeS() const
	{
		return currentTimeS;
	}

	double PlaybackController::getPlaybackRate() const
	{
		return playbackRate;
	}

	bool PlaybackController::isPlaying() const
	{
		return playing;
	}
}