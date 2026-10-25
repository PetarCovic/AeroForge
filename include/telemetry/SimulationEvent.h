#pragma once

#include <string>

namespace aeroforge::telemetry
{
	enum class SimulationEventType
	{
		EngineIgnition,
		EngineBurnout,
		SensorFaultActivated,
		SensorFaultCleared,
		ActuatorFaultActivated,
		ActuatorFaultCleared,
		ControllerSaturation,
		SimulationTermination,
		Custom
	};

	struct SimulationEvent
	{
		double timeS;
		SimulationEventType type;
		std::string message;
	};
}