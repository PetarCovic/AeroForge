#include <gtest/gtest.h>

#include <telemetry/EventLogger.h>
#include <telemetry/SimulationEvent.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::telemetry
{
	TEST(EventLoggerTests, StartsEmpty)
	{
		EventLogger logger;

		EXPECT_TRUE(logger.empty());
		EXPECT_EQ(logger.size(), 0u);
	}


	TEST(EventLoggerTests, RecordsEvent)
	{
		EventLogger logger;

		logger.log(
			SimulationEvent{
				1.5,
				SimulationEventType::EngineIgnition,
				"Engine ignition"
			}
		);

		ASSERT_EQ(
			logger.size(),
			1u
		);

		EXPECT_DOUBLE_EQ(
			logger.getEvents()[0].timeS,
			1.5
		);

		EXPECT_EQ(
			logger.getEvents()[0].type,
			SimulationEventType::EngineIgnition
		);
	}


	TEST(EventLoggerTests, AllowsMultipleEventsAtSameTimestamp)
	{
		EventLogger logger;

		logger.log(
			{
				1.0,
				SimulationEventType::SensorFaultActivated,
				"GPS dropout"
			}
		);

		logger.log(
			{
				1.0,
				SimulationEventType::ControllerSaturation,
				"Controller saturated"
			}
		);

		EXPECT_EQ(
			logger.size(),
			2u
		);
	}


	TEST(EventLoggerTests, RejectsTimeGoingBackward)
	{
		EventLogger logger;

		logger.log(
			{
				2.0,
				SimulationEventType::Custom,
				"First"
			}
		);

		EXPECT_THROW(
			logger.log(
				{
					1.0,
					SimulationEventType::Custom,
					"Second"
				}
			),
			std::invalid_argument
		);

		EXPECT_EQ(
			logger.size(),
			1u
		);
	}


	TEST(EventLoggerTests, RejectsNegativeTime)
	{
		EventLogger logger;

		EXPECT_THROW(
			logger.log(
				{
					-1.0,
					SimulationEventType::Custom,
					"Invalid"
				}
			),
			std::invalid_argument
		);
	}


	TEST(EventLoggerTests, RejectsNonFiniteTime)
	{
		EventLogger logger;

		EXPECT_THROW(
			logger.log(
				{
					std::numeric_limits<double>::quiet_NaN(),
					SimulationEventType::Custom,
					"Invalid"
				}
			),
			std::invalid_argument
		);
	}


	TEST(EventLoggerTests, FiltersEventsByType)
	{
		EventLogger logger;

		logger.log(
			{
				1.0,
				SimulationEventType::EngineIgnition,
				"Ignition"
			}
		);

		logger.log(
			{
				2.0,
				SimulationEventType::ControllerSaturation,
				"Saturation"
			}
		);

		logger.log(
			{
				3.0,
				SimulationEventType::ControllerSaturation,
				"Saturation again"
			}
		);

		auto events =
			logger.getEventsByType(
				SimulationEventType::ControllerSaturation
			);

		ASSERT_EQ(
			events.size(),
			2u
		);

		EXPECT_DOUBLE_EQ(
			events[0].timeS,
			2.0
		);

		EXPECT_DOUBLE_EQ(
			events[1].timeS,
			3.0
		);
	}


	TEST(EventLoggerTests, ClearResetsLogger)
	{
		EventLogger logger;

		logger.log(
			{
				10.0,
				SimulationEventType::Custom,
				"Event"
			}
		);

		logger.clear();

		EXPECT_TRUE(logger.empty());

		EXPECT_NO_THROW(
			logger.log(
				{
					0.0,
					SimulationEventType::Custom,
					"Restart"
				}
			)
		);
	}
}