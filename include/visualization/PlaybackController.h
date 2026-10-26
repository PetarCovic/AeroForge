#pragma once

namespace aeroforge::visualization
{
	class PlaybackController
	{
	public:
		PlaybackController(double startTimeS, double endTimeS);

		void play();

		void pause();

		void setPlaybackRate(double playbackRate);

		void seek(double simulationTimeS);

		void update(double wallDeltaS);

		void restart();

		double getStartTimeS() const;

		double getEndTimeS() const;

		double getCurrentTimeS() const;

		double getPlaybackRate() const;

		bool isPlaying() const;

	private:
		double startTimeS;
		double endTimeS;
		double currentTimeS;
		double playbackRate;
		bool playing;
	};
}