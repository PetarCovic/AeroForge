#include <simulation/SimulationPacer.h>

#include <chrono>
#include <thread>
#include <cmath>
#include <stdexcept>

namespace aeroforge::simulation
{
	SimulationPacer::SimulationPacer()
	{
		running = false;
	}

	void SimulationPacer::start()
	{
		start_clock = std::chrono::steady_clock::now();
		running = true;
	}

	double SimulationPacer::paceTo(double simulatedElapsedS)
	{
		if (!running)
		{
			throw std::logic_error("SimulationPacer is not running");
		}

		if (!std::isfinite(simulatedElapsedS) || simulatedElapsedS < 0.0)
		{
			throw std::invalid_argument(
				"simulatedElapsedS must be finite and non-negative");
		}

		const auto target =
			start_clock +
			std::chrono::duration_cast<std::chrono::steady_clock::duration>(
				std::chrono::duration<double>(simulatedElapsedS));

		auto now = std::chrono::steady_clock::now();

		if (now >= target)
		{
			return std::chrono::duration<double>(
				now - target).count();
		}

		constexpr auto spinWindow =
			std::chrono::milliseconds(20);

		const auto sleepUntil =
			target - spinWindow;

		if (now < sleepUntil)
		{
			std::this_thread::sleep_until(sleepUntil);
		}

		do
		{
			now = std::chrono::steady_clock::now();
		} while (now < target);

		return 0.0;
	}

	double SimulationPacer::getElapsedWallTimeS() const
	{
		if (!running)
		{
			throw std::logic_error("SimulationPacer is not running");
		}

		return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_clock).count();
	}
}