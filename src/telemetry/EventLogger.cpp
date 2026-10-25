#include <telemetry/EventLogger.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::telemetry
{
	void EventLogger::log(const SimulationEvent& event)
	{
		if (!std::isfinite(event.timeS) || event.timeS < 0)
		{
			throw std::invalid_argument("event.timeS must be finite and non-negative");
		}

		if (!events.empty())
		{
			if (event.timeS < events.back().timeS)
			{
				throw std::invalid_argument("event.timeS must be greater than or equal to events.back().timeS");
			}
		}

		events.push_back(event);
	}

	const std::vector<SimulationEvent>& EventLogger::getEvents() const
	{
		return events;
	}

	std::vector<SimulationEvent> EventLogger::getEventsByType(SimulationEventType type) const
	{
		std::vector<SimulationEvent> output;

		for (const SimulationEvent& event : events)
		{
			if (event.type == type)
			{
				output.push_back(event);
			}
		}

		return output;
	}

	std::size_t EventLogger::size() const
	{
		return events.size();
	}

	bool EventLogger::empty() const
	{
		return events.empty();
	}

	void EventLogger::clear()
	{
		events.clear();
	}
}