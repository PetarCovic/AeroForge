#pragma once

#include <telemetry/SimulationEvent.h>

#include <cstddef>
#include <vector>

namespace aeroforge::telemetry
{
	class EventLogger
	{
	public:
		void log(const SimulationEvent& event);

		const std::vector<SimulationEvent>& getEvents() const;

		std::vector<SimulationEvent> getEventsByType(SimulationEventType type) const;

		std::size_t size() const;

		bool empty() const;

		void clear();

	private:
		std::vector<SimulationEvent> events;
	};
}