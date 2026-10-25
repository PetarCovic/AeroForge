#include <gtest/gtest.h>

#include <telemetry/FlightAnalyzer.h>
#include <telemetry/TelemetrySample.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace aeroforge::telemetry
{
	namespace
	{
		TelemetrySample makeFlightSample(
			double timeS,
			double downM,
			const math::Vector3& velocityN,
			const math::Vector3& omegaB)
		{
			return TelemetrySample{
				timeS,

				math::Vector3(
					0.0,
					0.0,
					downM
				),

				velocityN,

				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),

				omegaB,

				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),

				math::Vector3(),
				math::Vector3(),
				math::Vector3(),

				0.0,
				0.0,
				0.0,
				false
			};
		}
	}


	TEST(FlightAnalyzerTests, RejectsEmptyTelemetry)
	{
		EXPECT_THROW(
			FlightAnalyzer::analyze({}),
			std::invalid_argument
		);
	}


	TEST(FlightAnalyzerTests, ReportsSampleCount)
	{
		std::vector<TelemetrySample> samples{
			makeFlightSample(
				0.0,
				0.0,
				math::Vector3(),
				math::Vector3()
			),

			makeFlightSample(
				1.0,
				-10.0,
				math::Vector3(10.0, 0.0, 0.0),
				math::Vector3()
			)
		};

		FlightMetrics metrics =
			FlightAnalyzer::analyze(samples);

		EXPECT_EQ(
			metrics.sampleCount,
			2u
		);
	}


	TEST(FlightAnalyzerTests, ComputesDuration)
	{
		std::vector<TelemetrySample> samples{
			makeFlightSample(
				2.0,
				0.0,
				math::Vector3(),
				math::Vector3()
			),

			makeFlightSample(
				7.5,
				0.0,
				math::Vector3(),
				math::Vector3()
			)
		};

		FlightMetrics metrics =
			FlightAnalyzer::analyze(samples);

		EXPECT_DOUBLE_EQ(
			metrics.durationS,
			5.5
		);
	}


	TEST(FlightAnalyzerTests, ComputesMaximumAltitudeUsingNedConvention)
	{
		std::vector<TelemetrySample> samples{
			makeFlightSample(
				0.0,
				0.0,
				math::Vector3(),
				math::Vector3()
			),

			makeFlightSample(
				1.0,
				-100.0,
				math::Vector3(),
				math::Vector3()
			),

			makeFlightSample(
				2.0,
				-250.0,
				math::Vector3(),
				math::Vector3()
			),

			makeFlightSample(
				3.0,
				-200.0,
				math::Vector3(),
				math::Vector3()
			)
		};

		FlightMetrics metrics =
			FlightAnalyzer::analyze(samples);

		EXPECT_DOUBLE_EQ(
			metrics.maxAltitudeM,
			250.0
		);
	}


	TEST(FlightAnalyzerTests, ComputesMaximumSpeed)
	{
		std::vector<TelemetrySample> samples{
			makeFlightSample(
				0.0,
				0.0,
				math::Vector3(3.0, 4.0, 0.0),
				math::Vector3()
			),

			makeFlightSample(
				1.0,
				0.0,
				math::Vector3(6.0, 8.0, 0.0),
				math::Vector3()
			)
		};

		FlightMetrics metrics =
			FlightAnalyzer::analyze(samples);

		EXPECT_DOUBLE_EQ(
			metrics.maxSpeedMps,
			10.0
		);
	}


	TEST(FlightAnalyzerTests, ComputesMaximumAngularRate)
	{
		std::vector<TelemetrySample> samples{
			makeFlightSample(
				0.0,
				0.0,
				math::Vector3(),
				math::Vector3(0.0, 0.0, 1.0)
			),

			makeFlightSample(
				1.0,
				0.0,
				math::Vector3(),
				math::Vector3(0.0, 3.0, 4.0)
			)
		};

		FlightMetrics metrics =
			FlightAnalyzer::analyze(samples);

		EXPECT_DOUBLE_EQ(
			metrics.maxAngularRateRadS,
			5.0
		);
	}


	TEST(FlightAnalyzerTests, ComputesAccelerationFromVelocityDifferences)
	{
		std::vector<TelemetrySample> samples{
			makeFlightSample(
				0.0,
				0.0,
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3()
			),

			makeFlightSample(
				1.0,
				0.0,
				math::Vector3(3.0, 4.0, 0.0),
				math::Vector3()
			),

			makeFlightSample(
				2.0,
				0.0,
				math::Vector3(9.0, 12.0, 0.0),
				math::Vector3()
			)
		};

		FlightMetrics metrics =
			FlightAnalyzer::analyze(samples);

		// First interval: 5 m/s^2
		// Second interval: 10 m/s^2
		EXPECT_DOUBLE_EQ(
			metrics.maxAccelerationMps2,
			10.0
		);
	}


	TEST(FlightAnalyzerTests, OneSampleHasZeroAcceleration)
	{
		FlightMetrics metrics =
			FlightAnalyzer::analyze(
				{
					makeFlightSample(
						0.0,
						0.0,
						math::Vector3(
							5.0,
							0.0,
							0.0
						),
						math::Vector3()
					)
				}
			);

		EXPECT_DOUBLE_EQ(
			metrics.maxAccelerationMps2,
			0.0
		);
	}


	TEST(FlightAnalyzerTests, StoresFinalState)
	{
		TelemetrySample first =
			makeFlightSample(
				0.0,
				0.0,
				math::Vector3(),
				math::Vector3()
			);

		TelemetrySample second =
			makeFlightSample(
				1.0,
				-50.0,
				math::Vector3(
					10.0,
					20.0,
					30.0
				),
				math::Vector3()
			);

		second.positionN =
			math::Vector3(
				1.0,
				2.0,
				-50.0
			);

		FlightMetrics metrics =
			FlightAnalyzer::analyze(
				{
					first,
					second
				}
			);

		EXPECT_DOUBLE_EQ(
			metrics.finalPositionN.x,
			1.0
		);

		EXPECT_DOUBLE_EQ(
			metrics.finalPositionN.z,
			-50.0
		);

		EXPECT_DOUBLE_EQ(
			metrics.finalVelocityN.y,
			20.0
		);
	}
}