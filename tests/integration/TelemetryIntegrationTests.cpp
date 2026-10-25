#include <gtest/gtest.h>

#include <telemetry/CsvTelemetryWriter.h>
#include <telemetry/EventLogger.h>
#include <telemetry/FlightAnalyzer.h>
#include <telemetry/GncAnalyzer.h>
#include <telemetry/RunMetadata.h>
#include <telemetry/TelemetryRecorder.h>
#include <telemetry/TelemetrySample.h>
#include <telemetry/TelemetrySchedule.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <sstream>

namespace aeroforge::telemetry
{
	TEST(
		TelemetryIntegrationTests,
		RecordsExportsAndAnalyzesSyntheticFlight
	)
	{
		const double simulationDtS =
			0.01;

		const double telemetryPeriodS =
			0.1;

		TelemetryRecorder recorder;

		TelemetrySchedule schedule(
			telemetryPeriodS
		);

		EventLogger events;

		RunMetadata metadata(
			"integration-run",
			12345u,
			7u,
			67890u,
			simulationDtS,
			{
				{"massKg", 100.0},
				{"thrustScale", 1.0}
			}
		);

		events.log(
			{
				0.0,
				SimulationEventType::EngineIgnition,
				"Engine ignition"
			}
		);

		const int steps =
			501;

		for (int i = 0;
			i < steps;
			++i)
		{
			double timeS =
				i * simulationDtS;

			if (!schedule.shouldRecord(timeS))
			{
				continue;
			}

			double altitudeM =
				20.0 * timeS;

			double velocityMps =
				20.0;

			double error =
				0.20
				* std::exp(-timeS);

			bool saturated =
				timeS < 0.3;

			TelemetrySample sample{
				timeS,

				math::Vector3(
					0.0,
					0.0,
					-altitudeM
				),

				math::Vector3(
					0.0,
					0.0,
					-velocityMps
				),

				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),

				math::Vector3(
					0.0,
					0.0,
					error
				),

				math::Vector3(
					0.0,
					0.0,
					-altitudeM + 0.5
				),

				math::Vector3(
					0.0,
					0.0,
					-velocityMps
				),

				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),

				math::Vector3(
					0.0,
					0.0,
					error
				),

				math::Vector3(
					0.0,
					0.0,
					error
				),

				math::Vector3(
					0.0,
					0.0,
					2.0 * error
				),

				math::Vector3(
					0.0,
					0.0,
					1.8 * error
				),

				1.0,
				0.0,
				0.0,

				saturated
			};

			recorder.record(sample);
		}

		events.log(
			{
				5.0,
				SimulationEventType::SimulationTermination,
				"Simulation complete"
			}
		);


		ASSERT_FALSE(
			recorder.empty()
		);

		EXPECT_EQ(
			metadata.getRunId(),
			"integration-run"
		);

		EXPECT_EQ(
			events.size(),
			2u
		);


		FlightMetrics flight =
			FlightAnalyzer::analyze(
				recorder.getSamples()
			);

		EXPECT_NEAR(
			flight.maxAltitudeM,
			100.0,
			1e-9
		);

		EXPECT_NEAR(
			flight.maxSpeedMps,
			20.0,
			1e-9
		);


		GncMetrics gnc =
			GncAnalyzer::analyze(
				recorder.getSamples(),
				0.05
			);

		EXPECT_GT(
			gnc.peakAttitudeErrorRad,
			0.0
		);

		EXPECT_LT(
			gnc.rmsAttitudeErrorRad,
			gnc.peakAttitudeErrorRad
		);

		EXPECT_GT(
			gnc.saturationFraction,
			0.0
		);

		EXPECT_TRUE(
			gnc.settlingTimeS.has_value()
		);


		std::ostringstream output;

		CsvTelemetryWriter::write(
			output,
			recorder.getSamples()
		);

		std::string csv =
			output.str();

		EXPECT_NE(
			csv.find("time_s"),
			std::string::npos
		);

		EXPECT_NE(
			csv.find("controller_saturated"),
			std::string::npos
		);

		EXPECT_GT(
			csv.size(),
			100u
		);
	}
}