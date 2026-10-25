#include <gtest/gtest.h>

#include <telemetry/CsvTelemetryWriter.h>
#include <telemetry/TelemetrySample.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace aeroforge::telemetry
{
	namespace
	{
		TelemetrySample makeCsvSample(
			double timeS)
		{
			return TelemetrySample{
				timeS,

				math::Vector3(1.0, 2.0, 3.0),
				math::Vector3(4.0, 5.0, 6.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.1, 0.2, 0.3),

				math::Vector3(1.1, 2.1, 3.1),
				math::Vector3(4.1, 5.1, 6.1),
				math::Quaternion(0.99, 0.01, 0.02, 0.03),
				math::Vector3(0.11, 0.21, 0.31),

				math::Vector3(0.01, 0.02, 0.03),
				math::Vector3(1.0, 2.0, 3.0),
				math::Vector3(0.9, 1.9, 2.9),

				0.75,
				0.05,
				-0.03,

				true
			};
		}
	}


	TEST(CsvTelemetryWriterTests, WritesHeaderForEmptyTelemetry)
	{
		std::ostringstream output;

		CsvTelemetryWriter::write(
			output,
			{}
		);

		std::string csv =
			output.str();

		EXPECT_NE(
			csv.find("time_s"),
			std::string::npos
		);

		EXPECT_NE(
			csv.find("position_n_m"),
			std::string::npos
		);

		EXPECT_NE(
			csv.find("attitude_q_w"),
			std::string::npos
		);

		EXPECT_NE(
			csv.find("throttle"),
			std::string::npos
		);

		EXPECT_NE(
			csv.find("controller_saturated"),
			std::string::npos
		);
	}


	TEST(CsvTelemetryWriterTests, WritesOneRowPerSample)
	{
		std::vector<TelemetrySample> samples{
			makeCsvSample(0.0),
			makeCsvSample(0.1),
			makeCsvSample(0.2)
		};

		std::ostringstream output;

		CsvTelemetryWriter::write(
			output,
			samples
		);

		std::string csv =
			output.str();

		std::size_t newlineCount =
			static_cast<std::size_t>(
				std::count(
					csv.begin(),
					csv.end(),
					'\n'
				)
				);

		// Header + three samples.
		EXPECT_EQ(
			newlineCount,
			4u
		);
	}


	TEST(CsvTelemetryWriterTests, IncludesSampleValues)
	{
		std::ostringstream output;

		CsvTelemetryWriter::write(
			output,
			{
				makeCsvSample(0.25)
			}
		);

		std::string csv =
			output.str();

		EXPECT_NE(
			csv.find("0.25"),
			std::string::npos
		);

		EXPECT_NE(
			csv.find("0.75"),
			std::string::npos
		);
	}


	TEST(CsvTelemetryWriterTests, WritesBooleanAsZeroOrOne)
	{
		std::ostringstream output;

		CsvTelemetryWriter::write(
			output,
			{
				makeCsvSample(0.0)
			}
		);

		std::string csv =
			output.str();

		EXPECT_NE(
			csv.find(",1\n"),
			std::string::npos
		);
	}


	TEST(CsvTelemetryWriterTests, PreservesHighPrecisionValues)
	{
		TelemetrySample sample =
			makeCsvSample(0.0);

		sample.positionN.x =
			1.23456789012345;

		std::ostringstream output;

		CsvTelemetryWriter::write(
			output,
			{ sample }
		);

		EXPECT_NE(
			output.str().find(
				"1.23456789012345"
			),
			std::string::npos
		);
	}


	TEST(CsvTelemetryWriterTests, ThrowsIfOutputStreamFails)
	{
		std::ostringstream output;

		output.setstate(
			std::ios::badbit
		);

		EXPECT_THROW(
			CsvTelemetryWriter::write(
				output,
				{
					makeCsvSample(0.0)
				}
			),
			std::runtime_error
		);
	}
}